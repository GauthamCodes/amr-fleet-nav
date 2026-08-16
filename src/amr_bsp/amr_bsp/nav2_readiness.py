"""Decide when a namespaced Nav2 stack is actually ready to be sent a goal.

WHY THIS IS NOT ``wait_for_server``

    A Nav2 server is a MANAGED (lifecycle) node. It creates its action server when
    it is CONFIGURED and only begins accepting goals when it is ACTIVATED. Between
    those two transitions ``ActionClient.wait_for_server()`` returns True against a
    server that will refuse every goal it is handed, answering with

        [bt_navigator] Action server is inactive. Rejecting the goal.

    and the mission script sees an immediate REJECTED with t_goal 0.0.

    The window is narrow when one robot comes up on an idle machine, which is why
    it never showed in a headless evidence run. It widens to seconds whenever the
    machine is busy - the Gazebo GUI attached, RViz drawing, two robot stacks
    activating twelve servers between them - which is to say it opens exactly when
    somebody is watching. Two separate mission scripts shipped this race.

    Asking the lifecycle node what state it is in removes the guess.

USAGE

    The check is POLLED rather than awaited, because callers run it from inside a
    timer callback where blocking on a future would deadlock the executor:

        self.readiness = Nav2Readiness(self, ["amr1", "amr2"])
        ...
        if not self.readiness.all_active():
            return          # try again on the next tick
"""

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState

#: The lifecycle node whose state decides whether a NavigateToPose goal is
#: accepted. Its siblings activate around it, but this is the one that answers.
GATEKEEPER = "bt_navigator"


class Nav2Readiness:
    """Poll each robot's ``bt_navigator`` until it reports the ACTIVE state."""

    def __init__(self, node, robots, server=GATEKEEPER):
        """Create one lifecycle state client per robot.

        Args:
            node: The rclpy node to create clients on.
            robots: Robot namespaces, e.g. ``["amr1", "amr2"]``.
            server: The managed node to interrogate.
        """
        self._node = node
        self._clients = {
            robot: node.create_client(GetState, f"/{robot}/{server}/get_state")
            for robot in robots
        }
        self._futures = {}
        self._active = set()

    @property
    def pending(self):
        """Return the robots not yet known to be active, sorted."""
        return sorted(set(self._clients) - self._active)

    def all_active(self):
        """Return True once every robot has answered ACTIVE at least once.

        Never blocks. Each call starts a query for any robot without one in
        flight and harvests whichever answers have arrived since the last call, so
        a robot that answers "still configuring" is simply asked again next tick.
        """
        for robot, client in self._clients.items():
            if robot in self._active:
                continue
            future = self._futures.get(robot)
            if future is None:
                if client.service_is_ready():
                    self._futures[robot] = client.call_async(GetState.Request())
                continue
            if not future.done():
                continue
            self._futures[robot] = None
            result = future.result()
            if result is not None and (
                result.current_state.id == State.PRIMARY_STATE_ACTIVE
            ):
                self._active.add(robot)
        return not self.pending
