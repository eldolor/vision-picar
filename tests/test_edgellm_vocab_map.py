"""3.78 amendment 5: the merged reduced vocabulary keeps every forced id."""

import pytest

from tools.jetson.edgellm_vocab_map import merge


def test_forced_ids_are_kept_and_size_is_a_multiple():
    base = list(range(0, 200, 2))          # 100 ids
    forced = [1, 3, 5, 151643]
    pad = list(range(1000, 1100))
    ids = merge(base, pad, forced)
    assert set(forced) <= set(ids) and set(base) <= set(ids)
    assert len(ids) % 128 == 0 and ids == sorted(ids)


def test_pad_running_out_is_an_error():
    with pytest.raises(ValueError):
        merge(list(range(5)), [10], [])
