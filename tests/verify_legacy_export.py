#!/usr/bin/env python3
"""Verify a PNG export against its source bag and an optional old dataset."""
import argparse
import json
from pathlib import Path
import re
import sys

import cv2
import numpy as np
import rosbag
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'recorder'))
from capture import GROUP, frame_groups
from core import image_array


def verify(bag_path, directory, reference=None):
    directory=Path(directory)
    rows=[json.loads(line) for line in (directory/'frames.jsonl').read_text().splitlines()]
    largest_error=0.;valid_pixels=0;checked=0
    shapes=set()
    with rosbag.Bag(str(bag_path)) as bag:
        for ns,group in frame_groups(bag):
            row=rows[checked]
            assert row['frame_index']==checked
            assert re.fullmatch(r'\d+_\d{9}\.png',Path(row['depth_path']).name)
            assert Path(row['rgb_path']).name==Path(row['depth_path']).name
            seconds,nanoseconds=divmod(ns,1000000000)
            assert Path(row['depth_path']).name=='%d_%09d.png'%(seconds,nanoseconds)
            raw=(directory/row['depth_path']).read_bytes()
            assert raw[:8]==b'\x89PNG\r\n\x1a\n' and raw[24]==16 and raw[25]==0
            depth=cv2.imread(str(directory/row['depth_path']),cv2.IMREAD_UNCHANGED)
            rgb=cv2.imread(str(directory/row['rgb_path']),cv2.IMREAD_UNCHANGED)
            assert depth.dtype==np.uint16 and depth.ndim==2
            assert depth.shape==rgb.shape[:2]
            shapes.add(tuple(depth.shape))
            source=group[GROUP[1]];meta=json.loads(group[GROUP[-1]].data)
            meters=image_array(source).astype(np.float64)*meta['depth_scale_m']
            with np.errstate(invalid='ignore'):
                valid=np.isfinite(meters)&(meters>0)
            # The projector's 0.1--60 m output lies within uint16 millimeters.
            assert ((meters[valid]>=.001)&(meters[valid]<=65.535)).all()
            assert np.array_equal(depth>0,valid)
            error=np.max(np.abs(depth[valid].astype(np.float64)/1000.-meters[valid]))
            largest_error=max(largest_error,float(error))
            assert error<=.0005001
            assert row['source_depth_encoding']==source.encoding
            assert row['source_depth_scale_m']==meta['depth_scale_m']
            assert row['depth_encoding']=='16UC1' and row['depth_scale_m']==.001
            expected_rgb=np.array(image_array(group[GROUP[0]]))
            if group[GROUP[0]].encoding=='rgb8':expected_rgb=cv2.cvtColor(expected_rgb,cv2.COLOR_RGB2BGR)
            elif group[GROUP[0]].encoding=='rgba8':expected_rgb=cv2.cvtColor(expected_rgb,cv2.COLOR_RGBA2BGRA)
            assert np.array_equal(rgb,expected_rgb)
            valid_pixels+=int(valid.sum());checked+=1
    assert checked==len(rows)==len(list((directory/'rgb').glob('*.png')))==len(list((directory/'depth_camera').glob('*.png')))
    report=dict(status='PASS',frames=checked,shapes=[list(s) for s in sorted(shapes)],
                depth_file_format='16-bit grayscale PNG',depth_dtype='uint16',depth_unit='millimeter',
                invalid_value=0,matching_rgb_depth_names=True,rgb_pixels_preserved=True,
                depth_valid_mask_preserved=True,valid_pixels_checked=valid_pixels,
                max_depth_quantization_error_m=largest_error,
                output_directory=str(directory.resolve()),source_bag=str(Path(bag_path).resolve()))
    if reference:
        reference=Path(reference)
        example=next(iter(sorted((reference/'depth_camera').glob('*.png'))))
        old=cv2.imread(str(example),cv2.IMREAD_UNCHANGED)
        assert old.dtype==np.uint16 and tuple(old.shape) in shapes
        old_calibration=yaml.safe_load((reference/'camera_intrinsics.yaml').read_text())
        new_calibration=yaml.safe_load((directory/'camera_intrinsics.yaml').read_text())
        assert old_calibration==new_calibration
        report.update(reference_dataset=str(reference),legacy_depth_dtype_matches=True,
                      legacy_dimensions_match=True,legacy_camera_intrinsics_match=True)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag');parser.add_argument('directory')
    parser.add_argument('--reference',help='Old dataset root containing depth_camera/ and camera_intrinsics.yaml')
    parser.add_argument('--report',help='Write the verification JSON to this file')
    args=parser.parse_args()
    result=verify(args.bag,args.directory,args.reference)
    output=json.dumps(result,indent=2)+'\n'
    if args.report:Path(args.report).write_text(output)
    print(output,end='')
