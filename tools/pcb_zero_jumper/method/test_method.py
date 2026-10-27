"""Analytic controls for the independent geometry and necessary-bound gates."""
import json,math
from geometry import component_labels,touches,polygon_contains
from preflight import tracks_in_channel,channel_verdict,alternating_pairs

def run():
    tests=[]
    def test(name,condition):
        assert condition,name;tests.append({'test':name,'pass':True})
    c=lambda x,y,r=1:{'kind':'circle','point':(x,y),'radius':r}
    t=lambda a,b,r=.6:{'kind':'segment','a':a,'b':b,'radius':r}
    poly={'kind':'polygon','outer':[(0,0),(10,0),(10,10),(0,10)],'holes':[[(3,3),(7,3),(7,7),(3,7)]]}
    test('same_net_labels_do_not_connect_disjoint_copper',len(set(component_labels([c(0,0),c(10,0)])))==2)
    test('track_connects_two_annuli',len(set(component_labels([c(0,0),t((0,0),(10,0)),c(10,0)])))==1)
    test('gap_is_not_filled_by_name',not touches(t((0,0),(3,0)),t((5,0),(8,0))))
    test('crossing_copper_is_connected',touches(t((0,0),(4,4)),t((0,4),(4,0))))
    test('collinear_join_is_connected',touches(t((0,0),(2,0)),t((2,0),(4,0))))
    test('zone_hole_is_not_copper',not polygon_contains((5,5),poly) and not touches(c(5,5,.5),poly))
    test('zone_solid_is_copper',polygon_contains((1,1),poly) and touches(c(1,1,.5),poly))
    test('pad_can_connect_at_zone_boundary',touches(c(3,5,.6),poly))
    test('separate_zone_islands_remain_separate',len(set(component_labels([{'kind':'polygon','outer':[(0,0),(1,0),(1,1),(0,1)],'holes':[]},{'kind':'polygon','outer':[(2,0),(3,0),(3,1),(2,1)],'holes':[]}])))==2)
    test('between_header_pads_rejected',2.54-2 < 1.2+2*.5)
    test('channel_capacity_three',tracks_in_channel(6)==3 and not channel_verdict(6,3)['rejected_for_this_fixed_channel'])
    test('forced_four_nets_rejected',channel_verdict(6,4)['rejected_for_this_fixed_channel'])
    test('alternating_boundary_pairs_rejected',alternating_pairs(['A1','B1','A2','B2'],['A1','A2'],['B1','B2']))
    test('nonalternating_boundary_pairs_not_rejected',not alternating_pairs(['A1','A2','B1','B2'],['A1','A2'],['B1','B2']))
    return tests

if __name__=='__main__':
    print(json.dumps({'analytic_controls':run()},indent=2))
