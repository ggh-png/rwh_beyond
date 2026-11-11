"""InferenceStages operations, independent of BT command dispatch."""
from rwh_beyond_msgs.action import InferGrasps

class InferenceStages:
    def infer_grasps(self,context):
        self.stage('infer_grasps_from_local_cloud')
        self.wait(self.infer.server_is_ready,10)
        goal=InferGrasps.Goal(input_mode=InferGrasps.Goal.POINT_CLOUD,object_cloud=context['cloud'],
            gripper_name='piper_hand',num_grasps=self.get_parameter('num_grasps').value,
            top_k=self.get_parameter('top_k').value)
        sent=self.infer.send_goal_async(goal)
        # Resolve acceptance even when cancellation arrives during the handshake.
        self.wait(sent.done,10,check_cancel=False);inference=sent.result()
        if not inference.accepted:raise RuntimeError('Grasp inference goal rejected')
        response=inference.get_result_async()
        try:self.wait(response.done,70)
        except Exception:
            inference.cancel_goal_async();raise
        result=response.result().result
        if not result.success or not result.message.startswith('OK:'):raise RuntimeError('Real GraspGenX inference failed: '+result.message)
        return {'candidates':result.candidates}

