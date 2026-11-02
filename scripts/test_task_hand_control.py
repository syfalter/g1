"""Mock DDS only: no socket, subprocess, serial access or motor commands."""
import threading
import os
import time
import unittest
from unittest.mock import patch,Mock
import xml.etree.ElementTree as ET
from task_hand_control import HandControl, POSTURES, model_joints, model_feedback, require_state
from task_hand_completion import transition


class FakeHand:
    def __init__(self):
        self.now=0.;self.calls=[];self.q=[1.]*6;self.freeze=False;self.stuck=False

    def sleep(self,t):self.now+=t

    def send(self,command):
        self.calls.append(command)
        op,side,_id,*q=command.split()
        if op=='set' and not self.stuck:self.q=list(map(float,q))
        if op=='get':
            return dict(code=0,hand=side,online=True,feedback_q=self.q[:],target_q=self.q[:],
                        error=[0]*6,temperature=[30]*6,timestamp_ms=1 if self.freeze else int(self.now*1000)+1)
        return {'code':0}


class HandTests(unittest.TestCase):
    def setUp(self):
        self.fake=FakeHand()
        self.hand=HandControl('no-network',enabled=True,transport=self.fake.send,
                              clock=lambda:self.fake.now,sleep=self.fake.sleep)

    def test_construct_is_nonactuating(self):
        self.assertEqual(self.fake.calls,[])
        self.assertIsNone(self.hand.process)

    def test_peer_preserves_interface_and_does_not_change_process_environment(self):
        config='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.123.99" /></Interfaces></General></Domain></CycloneDDS>'
        with patch.dict(os.environ,{'CYCLONEDDS_URI':config}):
            self.hand.peer_address='192.168.123.164'
            result=self.hand.dds_environment()
            root=ET.fromstring(result['CYCLONEDDS_URI'])
            self.assertEqual(root.find('.//NetworkInterface').get('address'),'192.168.123.99')
            self.assertEqual(root.find('.//Peer').get('Address'),'192.168.123.164')
            self.assertEqual(root.find('.//ParticipantIndex').text,'none')
            self.assertEqual(os.environ['CYCLONEDDS_URI'],config)
        self.assertEqual(self.fake.calls,[])

    def test_transport_timeout_is_not_service_rejection_and_never_retries_command(self):
        calls=[]
        def fail(command):calls.append(command);return {'code':3104}
        self.hand.transport=fail
        with self.assertRaisesRegex(TimeoutError,'no service acknowledgement'):
            self.hand.read('left')
        self.assertEqual(len(calls),1)

    def test_rpc_timeout_discards_client_and_stale_reply_without_replaying(self):
        for operation in ('get','set','hold'):
            process=Mock();process.poll.return_value=0
            self.hand.process=process;self.hand.buffer=b'old reply\n'
            calls=[]
            self.hand.transport=lambda command:(calls.append(command) or {'code':3104})
            with self.assertRaises(TimeoutError):
                self.hand.rpc(operation,'left',POSTURES['open'] if operation=='set' else None)
            self.assertEqual(len(calls),1)
            self.assertIsNone(self.hand.process)
            self.assertEqual(self.hand.buffer,b'')
            process.stdin.close.assert_called_once()
            process.stdout.close.assert_called_once()

    def test_readonly_never_sends_targets_or_hold(self):
        self.hand.enabled=False
        self.hand.read('left')
        for op in ('set','hold'):
            with self.assertRaisesRegex(RuntimeError,'disabled'):
                self.hand.rpc(op,'left',POSTURES['open'])
        self.assertEqual(len(self.fake.calls),1)

    def test_selected_hand_only_and_wait_for_feedback(self):
        readings=self.hand.transition('left',POSTURES['index_only'],lambda:None,duration=1.)
        self.assertGreaterEqual(len(readings),3)
        self.assertTrue(all(c.split()[1]=='left' for c in self.fake.calls))
        self.assertTrue(self.fake.calls[-1].startswith('hold left'))
        self.assertGreaterEqual(self.fake.now,1.)
        self.assertLess(self.fake.now,2.)
        self.assertEqual(self.fake.q,list(POSTURES['index_only']))

    def test_ack_without_motion_times_out_and_holds(self):
        self.fake.stuck=True
        with self.assertRaises(TimeoutError):
            self.hand.transition('right',POSTURES['index_only'],lambda:None,timeout=3.)
        self.assertTrue(self.fake.calls[-1].startswith('hold right'))

    def test_command_feedback_policy_accepts_contact_angle_error(self):
        self.fake.stuck=True;self.fake.q=[.068,.031,.056,.110,.722,0.]
        target=[.068,.016,.033,.020,.719,0.]
        readings=transition(self.hand,'right',target,lambda:None,duration=1.,require_target=False)
        self.assertGreaterEqual(self.fake.now,1.1)
        self.assertLess(self.fake.now,2.)
        self.assertAlmostEqual(readings[-1]['feedback_q'][3],.110)
        self.assertGreater(abs(readings[-1]['feedback_q'][3]-target[3]),.03)
        sent=[c.split()[3:] for c in self.fake.calls if c.startswith('set right')]
        self.assertEqual(list(map(float,sent[-1])),target)
        self.assertTrue(self.fake.calls[-1].startswith('hold right'))

    def test_default_completion_still_requires_target(self):
        self.fake.stuck=True
        with self.assertRaisesRegex(TimeoutError,'did not settle'):
            transition(self.hand,'right',POSTURES['index_only'],lambda:None,timeout=3.)

    def test_four_fingers_first_then_thumb_half_second_each(self):
        samples=[]
        def event(name, **details):
            if name == 'target_requested':samples.append((self.fake.now,details['target_q']))
        transition(self.hand,'right',[0.]*6,lambda:None,require_target=False,duration=1.,
                   finger_sequence='four_then_thumb',on_event=event)
        first=[q for t,q in samples if .05<t<.5]
        self.assertTrue(first)
        self.assertTrue(all(q[4:]==[1.,1.] for q in first))
        self.assertTrue(any(0.<q[0]<1. for q in first))
        second=[q for t,q in samples if .5<t<1.]
        self.assertTrue(second)
        self.assertTrue(all(q[:4]==[0.]*4 for q in second))
        self.assertTrue(any(0.<q[4]<1. for q in second))
        self.assertEqual(samples[-1][1],[0.]*6)
        self.assertLess(self.fake.now,1.5)

    def test_staged_hand_accepts_contact_offset_but_not_stale_feedback(self):
        self.fake.stuck=True
        transition(self.hand,'right',[0.]*6,lambda:None,require_target=False,duration=1.,
                   finger_sequence='four_then_thumb')
        self.assertEqual(self.fake.q,[1.]*6)
        self.fake.freeze=True
        with self.assertRaisesRegex(RuntimeError,'not advancing'):
            transition(self.hand,'right',[0.]*6,lambda:None,require_target=False,duration=1.,
                       finger_sequence='four_then_thumb')

    def test_release_thumb_before_four_fingers(self):
        self.fake.q=[0.]*6
        samples=[]
        def event(name, **details):
            if name == 'target_requested':samples.append((self.fake.now,details['target_q']))
        transition(self.hand,'right',[1.]*6,lambda:None,require_target=False,duration=1.,
                   finger_sequence='thumb_then_four',on_event=event)
        first=[q for t,q in samples if .05<t<.5]
        second=[q for t,q in samples if .5<t<1.]
        self.assertTrue(first and second)
        self.assertTrue(all(q[:4]==[0.]*4 for q in first))
        self.assertTrue(any(0.<q[4]<1. for q in first))
        self.assertTrue(all(q[4:]==[1.]*2 for q in second))
        self.assertTrue(any(0.<q[0]<1. for q in second))
        self.assertEqual(samples[-1][1],[1.]*6)
        self.assertLess(self.fake.now,1.5)

    def test_invalid_staged_policy_sends_nothing(self):
        for kwargs in (dict(require_target=True,duration=1.),dict(require_target=False,duration=2.)):
            with self.assertRaises(ValueError):
                transition(self.hand,'right',[0.]*6,lambda:None,finger_sequence='four_then_thumb',**kwargs)
        self.assertEqual(self.fake.calls,[])

    def test_command_feedback_policy_still_rejects_stale_feedback(self):
        self.fake.freeze=True
        with self.assertRaisesRegex(RuntimeError,'not advancing'):
            transition(self.hand,'right',POSTURES['open'],lambda:None,require_target=False)
        self.assertTrue(self.fake.calls[-1].startswith('hold right'))

    def test_command_feedback_policy_still_checks_stop(self):
        def check():
            if self.fake.now>.3:raise RuntimeError('STOP')
        with self.assertRaisesRegex(RuntimeError,'STOP'):
            transition(self.hand,'right',POSTURES['open'],check,require_target=False)
        self.assertTrue(self.fake.calls[-1].startswith('hold right'))
        self.assertLess(self.fake.now,1.)

    def test_command_feedback_policy_rejects_fault_and_temperature(self):
        for key,value in (('error',1),('temperature',60)):
            def send(command):
                result=self.fake.send(command)
                if command.startswith('get'):result[key]=[value]*6
                return result
            self.hand.transport=send
            with self.assertRaisesRegex(RuntimeError,'fault/temperature'):
                transition(self.hand,'right',POSTURES['open'],lambda:None,require_target=False)
        self.assertFalse(any(c.startswith('set') for c in self.fake.calls))

    def test_command_feedback_policy_requires_command_ack(self):
        def send(command):
            result=self.fake.send(command)
            return dict(code=3104) if command.startswith('set') else result
        self.hand.transport=send
        with self.assertRaisesRegex(TimeoutError,'no service acknowledgement'):
            transition(self.hand,'right',POSTURES['open'],lambda:None,require_target=False)
        self.assertTrue(self.fake.calls[-1].startswith('hold right'))

    def test_invalid_completion_policy_sends_nothing(self):
        with self.assertRaises(ValueError):
            transition(self.hand,'right',POSTURES['open'],lambda:None,require_target='false')
        self.assertEqual(self.fake.calls,[])

    def test_stale_feedback_rejected(self):
        self.fake.freeze=True
        with self.assertRaisesRegex(RuntimeError,'not advancing'):
            self.hand.transition('left',POSTURES['index_only'],lambda:None)
        self.assertTrue(self.fake.calls[-1].startswith('hold left'))

    def test_cancelled_worker_cannot_send_after_hold(self):
        def check():
            if self.fake.now>.3:raise RuntimeError('cancelled')
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            self.hand.transition('left',POSTURES['index_only'],check)
        last=len(self.fake.calls)
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            self.hand.rpc('set','left',POSTURES['open'],check)
        self.assertEqual(len(self.fake.calls),last)
        self.assertTrue(self.fake.calls[-1].startswith('hold left'))

    def test_hold_does_not_probe_unused_hand(self):
        self.hand.hold_active();self.assertEqual(self.fake.calls,[])
        self.hand.begin_motion(lambda:None)
        self.hand.rpc('set','right',POSTURES['open'])
        self.hand.hold_active()
        self.assertTrue(all('right' in c for c in self.fake.calls))

    def test_fault_and_temperature_do_not_clear_automatically(self):
        state=self.fake.send('get left 1')
        for key,value in (('error',1),('temperature',60)):
            bad=dict(state);bad[key]=[value]*6
            with self.assertRaisesRegex(RuntimeError,'fault/temperature'):require_state(bad,'left')
        self.assertFalse(any(c.startswith('clear') for c in self.fake.calls))

    def test_bilateral_mapping_and_roundtrip(self):
        for posture,q in POSTURES.items():
            left=model_joints('left',q);right=model_joints('right',q)
            self.assertEqual(list(left.values()),list(right.values()))
            self.assertEqual(model_feedback('left',left),q)
        self.assertAlmostEqual(model_joints('left',POSTURES['index_only'])['left_index_1_joint'],.15/1.1169)


class PipeCancellationTests(unittest.TestCase):
    def test_stop_interrupts_wait_discards_late_stream_and_confirms_hold(self):
        class PipeProcess:
            def __init__(self,reply=False):
                read,self.write_fd=os.pipe();self.stdout=os.fdopen(read,'rb',buffering=0)
                self.stdin=self;self.reply=reply;self.commands=[];self.sent=threading.Event();self.code=None

            def write(self,data):
                self.commands.append(data.decode().strip());self.sent.set()
                if self.reply:os.write(self.write_fd,b'TASK_HAND {"code":0}\n')

            def close(self):
                if self.write_fd is not None:os.close(self.write_fd);self.write_fd=None
                self.code=0

            def poll(self):return self.code
            def wait(self,**kw):self.close();return 0
            def terminate(self):self.close()
            def kill(self):self.close()

        first=PipeProcess();second=PipeProcess(reply=True)
        hand=HandControl('offline-pipe',enabled=True);hand.process=first
        failures=[]
        def operation():
            try:hand.rpc('set','left',POSTURES['index_only'])
            except Exception as error:failures.append(error)
        worker=threading.Thread(target=operation);worker.start()
        try:
            self.assertTrue(first.sent.wait(1))
            started=time.monotonic();hand.request_stop()
            with patch('task_hand_control.subprocess.Popen',return_value=second),\
                    patch.object(hand,'dds_environment',return_value={}),\
                    patch('task_hand_control.Path.is_file',return_value=True):
                hand.hold_active()
            worker.join(1)
            self.assertFalse(worker.is_alive())
            self.assertLess(time.monotonic()-started,1.)
            self.assertTrue(failures);self.assertIn('cancelled',str(failures[0]))
            self.assertEqual(len(first.commands),1)
            self.assertTrue(first.stdout.closed)
            self.assertEqual([c.split()[0] for c in second.commands],['hold'])
            self.assertFalse(hand.stop_pending.is_set());self.assertIsNone(hand.stop_error)
            with self.assertRaisesRegex(RuntimeError,'STOP'):hand.rpc('set','left',POSTURES['open'])
            self.assertEqual(len(second.commands),1)
        finally:
            hand.request_stop();worker.join(1);hand.close(abort=True)
            first.close();second.close()
            if not first.stdout.closed:first.stdout.close()
            if not second.stdout.closed:second.stdout.close()


if __name__=='__main__':unittest.main()
