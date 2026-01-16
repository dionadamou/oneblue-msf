# mqtt_logger.py
import sys
import queue
import threading
import paho.mqtt.client as mqtt
from datetime import datetime

class MQTTLogger:
    def __init__(self, topic="log/filtration", broker="10.0.46.111"): #set IP of orchestrator here
        self.topic = topic
        self.status_topic = "status/filtration"
        self.enabled = True
        self.log_queue = queue.Queue()
        try:
            self.client = mqtt.Client()
            self.client.connect(broker, 1883)
            self.client.loop_start()
            self.thread = threading.Thread(target=self._publish_loop, daemon=True)
            self.thread.start()
        except Exception as e:
            self.enabled = False
            sys.__stdout__.write(f"[MQTTLogger ERROR] {e}\n")

    def write(self, message):
        if message.strip():
            timestamp = datetime.now().strftime("[%H:%M:%S]")
            full_msg = f"{timestamp} {message.strip()}"
            sys.__stdout__.write(full_msg + "\n")
            if self.enabled:
                self.log_queue.put(full_msg)

    def flush(self):
        pass

    def _publish_loop(self):
        while True:
            msg = self.log_queue.get()
            try:
                self.client.publish(self.topic, msg)
            except Exception as e:
                sys.__stdout__.write(f"[MQTTLogger ERROR] {e}\n")
                self.enabled = False

    def done(self, script_name):
        if self.enabled:
            try:
                self.client.publish(self.status_topic, f"DONE:{script_name}")
            except Exception as e:
                sys.__stdout__.write(f"[MQTTLogger ERROR - done()] {e}\n")
