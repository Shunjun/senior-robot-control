from senior_perception.vision.static_tf_binding import (
    Edge,
    StaticBinding,
    TransformSnapshot,
    body_from_camera,
)


def snapshot(x=1.0):
    return TransformSnapshot((Edge("base", "laser", 0, (x, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)),))


def test_standard_tf_geometry_uses_no_receipt_or_admission_gate():
    state = StaticBinding("base", "laser")
    assert state.observe(snapshot())
    generation = state.generation
    assert state.pair_valid(1.0, 2.0, generation=generation)
    matrix = body_from_camera(state.current.edges, "base", "laser")
    assert tuple(row[3] for row in matrix) == (1.0, 2.0, 3.0, 1.0)
    assert state.observe(snapshot())
    assert state.generation == generation
    assert not state.pair_valid(2.0, 1.0, generation=generation)


def test_missing_or_updated_tf_invalidates_old_work_and_recovers():
    state = StaticBinding("base", "laser")
    assert state.observe(snapshot())
    generation = state.generation
    state.revoke()
    assert not state.pair_valid(1.0, 2.0, generation=generation)
    assert state.observe(snapshot(4.0))
    assert not state.pair_valid(1.0, 2.0, generation=generation)
    assert state.pair_valid(1.0, 2.0, generation=state.generation)
    assert body_from_camera(state.current.edges, "base", "laser")[0][3] == 4.0
    assert not state.observe(TransformSnapshot(()))
    assert state.observe(snapshot())
