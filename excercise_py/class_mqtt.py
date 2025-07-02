import paho.mqtt.client as mqtt


class MQTTSubscriber:
    def __init__(self, client_id=None):
        """
        Initialize the MQTT subscriber.

        :param broker: MQTT broker address
        :param port: MQTT broker port
        :param topics: List of topics to subscribe to
        :param client_id: Optional client ID
        """
        self.broker = "broker.emqx.io"
        self.port = 1883
        topics = ["python/mqtt", "test/topic2", "test/topic3"]

        self.topics = topics if isinstance(topics, list) else [topics]
        self.client_id = client_id or "MQTTSubscriber"
        self.client = mqtt.Client(client_id=self.client_id)

        # Attach callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        """
        Callback when the client connects to the broker.
        """
        if rc == 0:
            print("Connected successfully")
            for topic in self.topics:
                client.subscribe(topic)
                # print(f"Subscribed to topic: {topic}")
        else:
            print(f"Connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        """
        Callback when a message is received.
        """
        # print(f"Message received on topic {msg.topic}: {msg.payload.decode()}")
        if msg.topic == "python/mqtt":
            self.test_demo(msg.payload.decode())

    def test_demo(self, msg):
        print("msg", msg)

    def start(self):
        """
        Connect to the broker and start the loop.
        """
        self.client.connect(self.broker, self.port)
        print(f"Connecting to {self.broker}:{self.port}")
        self.client.loop_forever()


if __name__ == "__main__":

    subscriber = MQTTSubscriber()
    subscriber.start()
