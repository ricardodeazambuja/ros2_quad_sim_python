from threading import Lock, Thread
from rclpy.callback_groups import ReentrantCallbackGroup

class SyncROS2Messages:
    def __init__(self, node, topic_dict, func_process_msgs, max_delta_t=0.1):
        # topic_dict={"label":{"msg_type": ros_msg_type, "topic_name": "topic"}}
        self._msgs = {label:None for label in topic_dict.keys()}
        self._subscriber_recv = {label: False for label in topic_dict.keys()}
        self._process_lock = Lock()
        self.stop = False
        self.max_delta_t = max_delta_t

        self._process_thread = Thread(target=self._wait2process, args=(func_process_msgs,))

        self.subscribers = []
        for label in topic_dict.keys():
            msg_type = topic_dict[label]["msg_type"]
            topic = topic_dict[label]["topic_name"]
            sub = node.create_subscription(
                msg_type,
                topic,
                self._callback_gen(label),
                1, callback_group=ReentrantCallbackGroup())
            self.subscribers.append(sub)

        self._process_thread.start()


    def _wait2process(self, func_process_msgs):
        while not self.stop:
            if self._process_lock.acquire(blocking=True, timeout=self.max_delta_t):
                if all(self._subscriber_recv.values()):
                    for key in self._subscriber_recv.keys():
                        self._subscriber_recv[key] = False
                    func_process_msgs(self._msgs)
                self._process_lock.release()


    def _callback_gen(self, label):
        def callback(msg):
            if self._process_lock.acquire(blocking=True, timeout=self.max_delta_t):
                self._msgs[label] = msg
                self._subscriber_recv[label] = True
                self._process_lock.release()
                
        return callback