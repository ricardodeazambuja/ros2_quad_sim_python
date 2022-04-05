from threading import Barrier, BrokenBarrierError, Event
from rclpy.callback_groups import ReentrantCallbackGroup

class SyncROS2Messages:
    def __init__(self, node, topic_dict, func_process_msgs, max_delta_t=0.1):
        # topic_dict={"label":{"msg_type": ros_msg_type, "topic_name": "topic"}}
        self.msgs = {label:None for label in topic_dict.keys()}
        self.process_msgs_ev = Event()
        self.barrier = Barrier(len(topic_dict), action=self.process_msgs(func_process_msgs), timeout=max_delta_t)

        self.subscribers = []
        for label in topic_dict.keys():
            msg_type = topic_dict[label]["msg_type"]
            topic = topic_dict[label]["topic_name"]
            sub = node.create_subscription(
                msg_type,
                topic,
                self.callback_gen(label),
                1, callback_group=ReentrantCallbackGroup())
            self.subscribers.append(sub)


    def callback_gen(self, label):
        def callback(msg):
            try:
                if not self.process_msgs_ev.is_set():
                    self.msgs[label] = msg
                    self.barrier.wait()
            except BrokenBarrierError:
                self.barrier.reset()
        return callback


    def process_msgs(self, in_func):
        def out_func():
            # block subscribers from receiving new messages
            self.process_msgs_ev.set()

            in_func(self.msgs)

            # free the subscribers to receive new msgs
            self.process_msgs_ev.clear()
        return out_func