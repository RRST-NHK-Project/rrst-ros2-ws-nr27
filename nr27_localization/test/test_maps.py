"""取り込んだ地図が launch から読める形で揃っているかのテスト."""
import os

import pytest
import yaml

MAPS = os.path.join(os.path.dirname(__file__), '..', 'maps')


@pytest.mark.parametrize('level', ['ground', 'l1'])
def test_map_yaml_points_to_existing_pgm(level):
    with open(os.path.join(MAPS, f'field_2027_{level}.yaml')) as f:
        meta = yaml.safe_load(f)
    pgm = os.path.join(MAPS, meta['image'])
    with open(pgm, 'rb') as f:
        assert f.read(2) == b'P5'
    assert meta['resolution'] == pytest.approx(0.05)


def test_source_is_recorded():
    # どの dev_ws の world から作ったか分からない地図は使わない
    with open(os.path.join(MAPS, 'SOURCE.md')) as f:
        text = f.read()
    assert 'dev_ws のコミット' in text
