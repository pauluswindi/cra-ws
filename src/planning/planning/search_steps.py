"""Drive the same search to completion or cooperatively between ROS callbacks."""


def complete(steps):
    """Keep the synchronous API and the incremental algorithm identical."""
    while True:
        try:
            next(steps)
        except StopIteration as result:
            return result.value
