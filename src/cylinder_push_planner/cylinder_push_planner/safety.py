"""Pure watchdog rules for active reobservation."""


def navigation_safety_fault(
    now: float,
    last_scan_received: float,
    navigation_started: float,
    scan_timeout: float,
    navigation_timeout: float,
) -> str | None:
    """Return a stop reason when navigation input or duration is unsafe."""

    if scan_timeout <= 0.0 or navigation_timeout <= 0.0:
        raise ValueError("watchdog timeouts must be positive")
    if last_scan_received <= 0.0:
        return "laser scan is missing during navigation"
    if now - last_scan_received > scan_timeout:
        return "laser scan became stale during navigation"
    if now - navigation_started > navigation_timeout:
        return "viewpoint navigation timed out"
    return None
