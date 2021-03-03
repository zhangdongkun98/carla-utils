import rospy, rostopic


class RepubField(object):
    def __init__(self):
        rospy.init_node('repub_field', anonymous=True)

        argv = rospy.myargv()
        args = argv[1:]

        input_topic = args[0]
        output_fields = args[1:]

        if not input_topic.startswith('/'):
            input_topic = rospy.get_namespace() + input_topic
        if input_topic.endswith('/'):
            input_topic = input_topic[:-1]


        input_class = None
        rate = rospy.Rate(50)
        while input_class is None and not rospy.is_shutdown():
            input_class, _, _ = rostopic.get_topic_class(input_topic)
            rate.sleep()

        rospy.loginfo('[repub_field] input topic is %s', input_topic)
        self.sub = rospy.Subscriber(input_topic, input_class, self.callback)
        self.pubs = []
        self.legal_fields = []

        input_class_instance = input_class()

        output_classes = []
        for field in output_fields:
            if hasattr(input_class_instance, field):
                output_class = type(getattr(input_class_instance, field))
                output_topic = input_topic + '/' + field
                rospy.loginfo('[repub_field] output topic is %s', output_topic)
                self.legal_fields.append(field)
                self.pubs.append( rospy.Publisher(output_topic, output_class, queue_size=1) )


    def callback(self, input_msg):
        for (field, publisher) in zip(self.legal_fields, self.pubs):
            publisher.publish( getattr(input_msg, field) )


if __name__ == '__main__':
    RepubField()
    rospy.spin()
