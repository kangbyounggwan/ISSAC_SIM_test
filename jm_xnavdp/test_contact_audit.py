import unittest
from types import SimpleNamespace as S
from jm_xnavdp.contact_audit import ContactAudit


class ContactAuditTests(unittest.TestCase):
    def audit(self,other,robot='/World/H1Fleet/H1_1/arm/collider'):
        a=ContactAudit.__new__(ContactAudit)
        a.robot='/World/H1Fleet/H1_1'; a.decode=str; a.pairs={}; a.details=[]; a.frames=0
        h=S(collider0=robot,collider1=other,num_contact_data=1,contact_data_offset=0)
        d=S(impulse=[0,0,0.01],separation=-.001,position=[1,2,3],normal=[0,0,1])
        a.interface=S(get_contact_report=lambda:([h],[d]))
        a.sample(.005)
        return a

    def test_floor_is_preserved_as_positive_sensor_evidence(self):
        a=self.audit('/World/Dadong/F2/slab')
        self.assertEqual(len(a.pairs),1)
        self.assertEqual(len(a.details),0)
        p=next(iter(a.pairs.values()))
        self.assertEqual(p['points'],1)
        self.assertAlmostEqual(p['max_impulse_Ns'],.01)

    def test_rack_and_workcell_contacts_are_preserved(self):
        for other in ['/World/Dadong/F2/Products/RACK_cont_008/deck','/World/JMRingWorkcells/H1_1/CantileverPlate']:
            a=self.audit(other)
            self.assertEqual(len(a.details),1)
            self.assertEqual(a.details[0]['separation_m'],-.001)

    def test_other_robot_not_attributed_to_h1_1(self):
        a=self.audit('/World/Dadong/F2/slab',robot='/World/H1Fleet/H1_2/arm/collider')
        self.assertEqual(a.pairs,{})
