"""Clearly labelled interim excerpt; retain and link the full failed rollout."""
from pathlib import Path
import argparse
import json
import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--end', type=float, default=13.)
    args = p.parse_args()
    source = args.run/'keti_detour_720p.mp4'
    target = args.run/'keti_detour_segment_INTERIM_720p.mp4'
    if target.exists():
        raise RuntimeError('Interim excerpt already exists')
    result = json.loads((args.run/'result.json').read_text())
    if result['pass'] or not result['failure'] or args.end >= result['time_s']:
        raise ValueError('This excerpt tool requires a disclosed failed full run and a shorter segment')
    reader = imageio.get_reader(str(source))
    meta = reader.get_meta_data()
    writer = imageio.get_writer(str(target), fps=24, codec='libx264', quality=8, macro_block_size=None)
    font = ImageFont.truetype('C:/Windows/Fonts/malgun.ttf', 19)
    frames = 0
    try:
        for n, frame in enumerate(reader):
            if n >= round(args.end*24):
                break
            img = Image.fromarray(frame)
            draw = ImageDraw.Draw(img)
            draw.rectangle((10,87,775,121), fill=(49,32,16))
            draw.text((21,91), '중간 검증: 우회 구간 발췌 | 전체 실행의 최종 정지는 실패',
                      fill=(255,224,161),font=font)
            writer.append_data(np.asarray(img))
            frames += 1
    finally:
        reader.close()
        writer.close()
    report = {'source_full_video':str(source.resolve()),'excerpt_video':str(target.resolve()),
              'excerpt_interval_s':[0,args.end], 'frames':frames,'width':meta['size'][0],
              'height':meta['size'][1],'fps':24,'full_run_success':False,
              'full_run_failure':result['failure'], 'final_stop_in_excerpt':False,
              'disclosure_burned_into_every_frame':True}
    (args.run/'interim_excerpt.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
