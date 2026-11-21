"""Read-only filtering over the completed, frozen reference positions."""

from senior_perception.learning_record import LearnedCatalog, LearningGeometry
from senior_perception.self_filter import FilteredObservation
from senior_perception.self_learning import SelfReturnModel


class LearningFilterView:
    def __init__(self, catalog: LearnedCatalog, geometry: LearningGeometry) -> None:
        self.catalog, self.geometry = catalog, geometry
        self._model = SelfReturnModel(geometry.physical_polygon)
        self._model.learned = catalog.points

    def filter(self, observation: FilteredObservation) -> FilteredObservation:
        return self._model.filter(observation, provisional=False)
