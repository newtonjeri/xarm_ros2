#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading

from xarm_msgs.action import VoiceCommand

from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractExceptionHandler

threading.Thread(target=lambda: rclpy.init()).start()
action_client = ActionClient(Node("alexa_interface_node"), VoiceCommand, "voice_command_server")

SKILL_ID = "amzn1.ask.skill.7836951c-8a46-4bca-a028-4fb850ae7ef9"

app = Flask(__name__)

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        print("Got LaunchRequest - %s" % str(handler_input))
        # type: (HandlerInput) -> Response
        speech_text = "x arm seven robot arm is ready to receive commands!"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)
        
        goal = VoiceCommand.Goal()
        goal.task_code = 0
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response
    

class MoveHomeIntentHandler(AbstractRequestHandler):
    launchRequestHandler = LaunchRequestHandler()
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("MoveHomeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "x arm seven robot arm moving to home pose"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("MoveHome", speech_text)).set_should_end_session(
            True)
        
        goal = VoiceCommand.Goal()
        goal.task_code = 1
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response
    
    
class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Moving to pick pose"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Picking", speech_text)).set_should_end_session(
            True)
        goal = VoiceCommand.Goal()
        goal.task_code = 2
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response
    

class PlacIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PlacIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Moving to place pose"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Placing", speech_text)).set_should_end_session(
            True)
        goal = VoiceCommand.Goal()
        goal.task_code = 2
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response
    

class OpenGripperIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("OpenGripperIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "opening gripper"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("OpenGripper", speech_text)).set_should_end_session(
            True)
        goal = VoiceCommand.Goal()
        goal.task_code = 4
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response
    

class CloseGripperIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("CloseGripperIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "closing gripper"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("CloseGripper", speech_text)).set_should_end_session(
            True)
        goal = VoiceCommand.Goal()
        goal.task_code = 5
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response
    

class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        # Log the exception in CloudWatch Logs
        print(exception)

        speech = "Sorry, I didn't get it. Can you please say it again!!"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response

    
skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(MoveHomeIntentHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(PlacIntentHandler())
skill_builder.add_request_handler(OpenGripperIntentHandler())
skill_builder.add_request_handler(CloseGripperIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())
# Register your intent handlers to the skill_builder object

skill_adapter = SkillAdapter(
    skill=skill_builder.create(), skill_id=SKILL_ID, app=app)

@app.route("/")
def invoke_skill():
    return skill_adapter.dispatch_request()


skill_adapter.register(app=app, route="/")

if __name__ == "__main__":
    app.run()