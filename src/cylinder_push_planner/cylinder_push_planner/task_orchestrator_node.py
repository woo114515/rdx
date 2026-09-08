"""Bundle Task 3 service sequences without bypassing stage safety checks."""

from __future__ import annotations

import json

import rclpy
from color_object_sorter_interfaces.srv import (
    SetObjectInventory,
    SetSnapshotExclusions,
)
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .task_workflow import (
    RUN_ONCE_COMPLETE_STATES,
    advance_services,
    run_once_service,
    validate_inventory,
)


class TaskOrchestratorNode(Node):
    """Expose one preparation call and one explicit call per motion stage."""

    def __init__(self) -> None:
        super().__init__("cylinder_task_orchestrator")
        self.declare_parameter("inventory_colors", ["blue", "green", "red"])
        self.declare_parameter("inventory_counts", [2, 2, 2])

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, "/cylinder_task/status", latched
        )
        self.create_subscription(
            String,
            "/cylinder_push_execution/status",
            self._on_execution_status,
            latched,
        )
        self._execution_state = "unknown"
        self._busy = False
        self._run_once_active = False
        self._run_once_service_pending = False
        self._run_once_consumed_state: str | None = None
        self._run_once_waiting_for_fresh_plan = False
        # Do not use ``_clients``: rclpy.Node owns that internal list.
        self._service_clients: dict[str, object] = {}
        for name in (
            "/cylinder_push_execution/reset_task",
            "/cylinder_push_plan/reset_task",
            "/cylinder_validation/reset",
            "/cylinder_snapshot/reset",
            "/cylinder_snapshot/start",
            "/cylinder_push_plan/generate",
            "/cylinder_push_execution/arm",
            "/cylinder_push_execution/start_approach",
            "/cylinder_push_execution/arm_contact",
            "/cylinder_push_execution/start_contact",
            "/cylinder_push_execution/arm_push",
            "/cylinder_push_execution/start_push",
            "/cylinder_push_execution/arm_return",
            "/cylinder_push_execution/start_return",
            "/cylinder_push_execution/finalize_delivery",
            "/cylinder_push_execution/cancel",
        ):
            self._service_clients[name] = self.create_client(Trigger, name)
        self._service_clients["/cylinder_snapshot/set_exclusions"] = self.create_client(
            SetSnapshotExclusions, "/cylinder_snapshot/set_exclusions"
        )
        self._service_clients["/cylinder_snapshot/set_inventory"] = self.create_client(
            SetObjectInventory, "/cylinder_snapshot/set_inventory"
        )

        self.create_service(Trigger, "/cylinder_task/prepare", self._prepare)
        self.create_service(Trigger, "/cylinder_task/generate", self._generate)
        self.create_service(Trigger, "/cylinder_task/advance", self._advance)
        self.create_service(Trigger, "/cylinder_task/run_once", self._run_once)
        self.create_service(Trigger, "/cylinder_task/cancel", self._cancel)
        self._publish("idle", "call /cylinder_task/prepare for a new field")

    def _on_execution_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        state = payload.get("state")
        if isinstance(state, str) and state:
            self._execution_state = state
            if (
                self._run_once_waiting_for_fresh_plan
                and state == "idle"
                and payload.get("reason") == "fresh complete push preview received"
            ):
                self._run_once_waiting_for_fresh_plan = False
                self._run_once_consumed_state = None
            self._drive_run_once()

    def _prepare(self, request, response):
        del request
        if self._busy:
            return self._failure(response, "another bundled operation is active")
        colors = [str(value) for value in self.get_parameter("inventory_colors").value]
        counts = [int(value) for value in self.get_parameter("inventory_counts").value]
        try:
            validate_inventory(colors, counts)
        except ValueError as error:
            return self._failure(response, str(error))

        exclusions = SetSnapshotExclusions.Request()
        exclusions.centers = []
        exclusions.radius = 0.0
        inventory = SetObjectInventory.Request()
        inventory.colors = colors
        inventory.counts = counts
        steps = [
            (
                "execution task reset",
                "/cylinder_push_execution/reset_task",
                Trigger.Request(),
            ),
            ("task-plan reset", "/cylinder_push_plan/reset_task", Trigger.Request()),
            ("validation reset", "/cylinder_validation/reset", Trigger.Request()),
            ("snapshot reset", "/cylinder_snapshot/reset", Trigger.Request()),
            ("exclusions clear", "/cylinder_snapshot/set_exclusions", exclusions),
            ("inventory update", "/cylinder_snapshot/set_inventory", inventory),
            ("snapshot start", "/cylinder_snapshot/start", Trigger.Request()),
        ]
        if not self._services_ready(name for _, name, _ in steps):
            return self._failure(response, "one or more preparation services are unavailable")
        self._busy = True
        self._publish("preparing", f"configuring inventory {dict(zip(colors, counts))}")
        self._run_steps(steps, "collecting", "snapshot collection started")
        response.success = True
        response.message = "preparation accepted; monitor /cylinder_task/status"
        return response

    def _generate(self, request, response):
        del request
        return self._start_trigger_operation(
            response,
            [("plan generation", "/cylinder_push_plan/generate", Trigger.Request())],
            "planned",
            "push preview generated; inspect RViz before advance",
        )

    def _advance(self, request, response):
        del request
        if self._busy:
            return self._failure(response, "another bundled operation is active")
        try:
            arm_name, start_name, stage = advance_services(self._execution_state)
        except ValueError as error:
            return self._failure(response, str(error))
        steps = [(f"{stage} arm", arm_name, Trigger.Request())]
        if start_name is not None:
            steps.append((f"{stage} start", start_name, Trigger.Request()))
        return self._start_trigger_operation(
            response,
            steps,
            f"{stage}_requested",
            f"{stage} accepted; monitor execution status",
        )

    def _run_once(self, request, response):
        del request
        if self._busy or self._run_once_active:
            return self._failure(response, "another bundled operation is active")
        if self._execution_state not in {"idle", "failed", "cancelled"}:
            return self._failure(
                response,
                f"run_once requires an idle executor, got {self._execution_state}",
            )
        required = {
            "/cylinder_push_plan/generate",
            "/cylinder_push_execution/arm",
            "/cylinder_push_execution/start_approach",
            "/cylinder_push_execution/arm_contact",
            "/cylinder_push_execution/start_contact",
            "/cylinder_push_execution/arm_push",
            "/cylinder_push_execution/start_push",
            "/cylinder_push_execution/arm_return",
            "/cylinder_push_execution/start_return",
            "/cylinder_push_execution/finalize_delivery",
            "/cylinder_push_execution/cancel",
        }
        if not self._services_ready(required):
            return self._failure(response, "one or more execution services are unavailable")
        self._busy = True
        self._run_once_active = True
        self._run_once_consumed_state = None
        self._run_once_waiting_for_fresh_plan = True
        self._publish(
            "run_once_active",
            "generating a fresh plan before automatic execution",
        )
        self._call_run_once_service(
            "/cylinder_push_plan/generate", "generate fresh plan"
        )
        response.success = True
        response.message = "one push cycle accepted; monitor /cylinder_task/status"
        return response

    def _cancel(self, request, response):
        del request
        self._run_once_active = False
        self._run_once_service_pending = False
        self._run_once_consumed_state = None
        self._run_once_waiting_for_fresh_plan = False
        if self._busy:
            self._busy = False
        client = self._service_clients["/cylinder_push_execution/cancel"]
        if not client.service_is_ready():
            return self._failure(response, "execution cancel service is unavailable")
        client.call_async(Trigger.Request())
        self._publish("cancelling", "execution cancel requested")
        response.success = True
        response.message = "cancel requested"
        return response

    def _drive_run_once(self) -> None:
        if not self._run_once_active or self._run_once_service_pending:
            return
        if self._run_once_waiting_for_fresh_plan:
            self._publish(
                "run_once_active", "waiting for the complete fresh plan"
            )
            return
        # Status publication and service responses are asynchronous. Do not
        # invoke the same boundary service twice while waiting for the state
        # caused by its first invocation.
        if self._execution_state == self._run_once_consumed_state:
            return
        self._run_once_consumed_state = self._execution_state
        if self._execution_state in RUN_ONCE_COMPLETE_STATES:
            completed_state = self._execution_state
            self._run_once_active = False
            self._busy = False
            self._publish(
                "run_once_complete",
                f"one push cycle completed: {completed_state}",
            )
            return
        if self._execution_state in {"failed", "cancelled"}:
            self._run_once_failed(
                f"executor stopped in state {self._execution_state}"
            )
            return
        try:
            next_service = run_once_service(self._execution_state)
        except ValueError as error:
            self._run_once_failed(str(error))
            return
        if next_service is None:
            self._publish(
                "run_once_active",
                f"waiting for executor state {self._execution_state}",
            )
            return
        name, label = next_service
        self._call_run_once_service(name, label)

    def _call_run_once_service(self, name: str, label: str) -> None:
        if not self._run_once_active:
            return
        client = self._service_clients[name]
        if not client.service_is_ready():
            self._run_once_failed(f"{label} service is unavailable")
            return
        self._run_once_service_pending = True
        self._run_once_consumed_state = self._execution_state
        self._publish("run_once_active", f"requesting {label}")
        future = client.call_async(Trigger.Request())

        def completed(done) -> None:
            if not self._run_once_active:
                return
            self._run_once_service_pending = False
            try:
                result = done.result()
            except Exception as error:
                self._run_once_failed(f"{label} failed: {error}")
                return
            if not result.success:
                self._run_once_failed(f"{label} rejected: {result.message}")
                return
            self._publish("run_once_active", f"{label}: {result.message}")
            # The execution-status message can arrive before this service
            # response. Re-evaluate the most recent state in either ordering.
            self._drive_run_once()

        future.add_done_callback(completed)

    def _run_once_failed(self, reason: str) -> None:
        self._run_once_active = False
        self._run_once_service_pending = False
        self._run_once_consumed_state = None
        self._run_once_waiting_for_fresh_plan = False
        self._busy = False
        cancel = self._service_clients["/cylinder_push_execution/cancel"]
        if cancel.service_is_ready():
            cancel.call_async(Trigger.Request())
        self.get_logger().error(reason)
        self._publish("run_once_failed", reason)

    def _start_trigger_operation(self, response, steps, state, reason):
        if self._busy:
            return self._failure(response, "another bundled operation is active")
        if not self._services_ready(name for _, name, _ in steps):
            return self._failure(response, "required service is unavailable")
        self._busy = True
        self._publish(f"requesting_{state}", reason)
        self._run_steps(steps, state, reason)
        response.success = True
        response.message = f"{state} accepted; monitor /cylinder_task/status"
        return response

    def _services_ready(self, names) -> bool:
        return all(self._service_clients[name].service_is_ready() for name in names)

    def _run_steps(self, steps, final_state: str, final_reason: str) -> None:
        def run(index: int) -> None:
            if not self._busy:
                return
            if index == len(steps):
                self._busy = False
                self._publish(final_state, final_reason)
                return
            label, name, request = steps[index]
            future = self._service_clients[name].call_async(request)

            def completed(done) -> None:
                if not self._busy:
                    return
                try:
                    result = done.result()
                except Exception as error:
                    self._operation_failed(f"{label} failed: {error}")
                    return
                if not result.success:
                    self._operation_failed(f"{label} rejected: {result.message}")
                    return
                self._publish("in_progress", f"{label}: {result.message}")
                run(index + 1)

            future.add_done_callback(completed)

        run(0)

    def _operation_failed(self, reason: str) -> None:
        self._busy = False
        self.get_logger().error(reason)
        self._publish("failed", reason)

    def _failure(self, response, reason: str):
        response.success = False
        response.message = reason
        self._publish("rejected", reason)
        return response

    def _publish(self, state: str, reason: str) -> None:
        payload = {
            "busy": self._busy,
            "execution_state": self._execution_state,
            "reason": reason,
            "state": state,
        }
        self._status_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskOrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
