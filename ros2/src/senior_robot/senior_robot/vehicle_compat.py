"""//Compatible: transitional attribute access for historical ROS consumers."""


class OwnedState:
    """Forward legacy reads/writes without storing business state on MotionCore."""

    def __init__(self, owner, attribute):
        self.owner = owner
        self.attribute = attribute

    def __get__(self, instance, owner=None):
        if instance is None:
            return self
        return getattr(getattr(instance, self.owner), self.attribute)

    def __set__(self, instance, value):
        setattr(getattr(instance, self.owner), self.attribute, value)
