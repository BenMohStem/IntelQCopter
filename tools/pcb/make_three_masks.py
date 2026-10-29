"""Place three unchanged vector PCB masks on A4, at PDF-native 1:1 scale."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import fitz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output_directory', type=Path)
    args = parser.parse_args()
    source = fitz.open(args.source)
    if len(source) != 1 or source[0].rotation:
        raise ValueError('Expected one unrotated PCB page')
    page = source[0]
    # Ignore the full-page white background, retain all remaining vector marks.
    bounds = fitz.Rect()
    for drawing in page.get_drawings():
        if drawing['rect'] == page.rect and drawing.get('fill') == (1., 1., 1.):
            continue
        bounds |= drawing['rect']
    if bounds.is_empty:
        raise ValueError('No vector artwork found')
    # Retain the outside half-width of the 0.72 pt board outline plus spare space.
    clip = fitz.Rect(bounds.x0 - .6, bounds.y0 - .6,
                     bounds.x1 + .6, bounds.y1 + .6) & page.rect
    mm = 72 / 25.4
    width, height = clip.width, clip.height
    gap = 10 * mm
    paper_width, paper_height = 210 * mm, 297 * mm
    if 2 * width + gap > paper_width - 20 * mm or 2 * height + gap > paper_height - 20 * mm:
        raise ValueError('Three full-size copies do not fit this A4 arrangement')
    left = (paper_width - 2 * width - gap) / 2
    top = (paper_height - 2 * height - gap) / 2
    positions = [(left, top), (left + width + gap, top),
                 ((paper_width - width) / 2, top + height + gap)]
    result = fitz.open()
    target = result.new_page(width=paper_width, height=paper_height)
    rects = []
    for x, y in positions:
        rect = fitz.Rect(x, y, x + width, y + height)
        target.show_pdf_page(rect, source, 0, clip=clip)
        assert abs(rect.width / clip.width - 1) < 1e-9
        assert abs(rect.height / clip.height - 1) < 1e-9
        rects.append(list(rect))
    result.xref_set_key(result.pdf_catalog(), 'ViewerPreferences', '<</PrintScaling /None>>')
    args.output_directory.mkdir(parents=True, exist_ok=True)
    output = args.output_directory / 'IntelQCopter-three-masks-A4-1to1.pdf'
    result.save(output, garbage=4, deflate=True)
    shutil.copyfile(args.source, args.output_directory / 'original-user-mask.pdf')
    reopened = fitz.open(output)
    assert len(reopened) == 1 and not reopened[0].get_text().strip()
    forms = reopened[0].get_xobjects()
    placements = [item for item in forms if item[2] == 0]
    assert len(placements) == 3
    for item in placements:
        matrix = reopened.xref_get_key(item[0], 'Matrix')[1]
        values = [float(x) for x in matrix.strip('[]').split()]
        assert values[:4] == [1., 0., 0., 1.], matrix
    manifest = {
        'source_sha256': hashlib.sha256(args.source.read_bytes()).hexdigest(),
        'output_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
        'copies': 3, 'paper_mm': [210, 297], 'scale': 1,
        'source_artwork_bounds_points': list(bounds),
        'source_clip_points': list(clip), 'placements_points': rects,
        'artwork_bounds_mm': [bounds.width / mm, bounds.height / mm],
        'orientation': 'unchanged; no additional mirror',
        'polarity': 'unchanged from supplied PDF',
        'checks': ['Three vector placements with identity scale matrices',
                   'No added text; original source retained',
                   'PrintScaling=None; printer must use Actual size/100 percent'],
        'limits': 'Print preparation only; no electrical design or original export scale certification'
    }
    (args.output_directory / 'mask-checks.json').write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
