"""Necessary geometric feasibility bounds, never a complete router."""
import argparse,json,math

def tracks_in_channel(width,track=1.2,clearance=.5,boundary_clearance=.5):
    # n*w+(n-1)*c+2*b <= W; only valid when every route MUST cross this channel.
    return max(0,math.floor((width-2*boundary_clearance+clearance+1e-9)/(track+clearance)))

def channel_verdict(width,demands,track=1.2,clearance=.5,boundary_clearance=.5):
    n=tracks_in_channel(width,track,clearance,boundary_clearance)
    return {'capacity_upper_bound':n,'forced_distinct_nets':demands,
            'rejected_for_this_fixed_channel':demands>n,
            'qualification':'Necessary cut bound only: requires proof that no alternative path around channel exists'}

def alternating_pairs(order,pair_a,pair_b):
    # For terminals on ONE simple closed boundary, within that region only.
    ia=sorted(order.index(q) for q in pair_a)
    ib=[order.index(q) for q in pair_b]
    return (ia[0]<ib[0]<ia[1]) != (ia[0]<ib[1]<ia[1])

def examples():
    return {'header_gap_mm':2.54-2.,'required_between_pad_gap_mm':1.2+2*.5,
            'between_header_pads_possible':2.54-2.>=1.2+2*.5,
            'six_mm_channel_three_nets':channel_verdict(6,3),
            'six_mm_channel_four_nets':channel_verdict(6,4),
            'alternating_closed_channel_ABAB':alternating_pairs(['A1','B1','A2','B2'],['A1','A2'],['B1','B2']),
            'nonalternating_AABB':alternating_pairs(['A1','A2','B1','B2'],['A1','A2'],['B1','B2']),
            'whole_board_feasibility':'Not determined by these necessary tests'}

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output');args=parser.parse_args()
    text=json.dumps(examples(),indent=2)+'\n'
    if args.output:
        from pathlib import Path
        Path(args.output).write_text(text,encoding='utf-8')
    print(text,end='')
