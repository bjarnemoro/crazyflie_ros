import rclpy
from rclpy.node import Node



import json
from google import genai
from barrier_msg.msg import  TMsgllm
from std_msgs.msg import String


SYSTEM_PROMPT = """
You translate natural language commands into robot task specifications according to the following scheme


Schema:
{
  "tasks": [
    {
      "rel_position": float[],
      "size": float,
      "period_num": int,
      "edges": int[],
      "operator": string,
      "timespan": [int, int]
    }
  ]
}

Output ONLY valid JSON.
Do NOT include explanations, markdown, or comments.

Rules:
- "edges" is a list of two integers indicating the indices of the two agents involved in the task. If the task involves a single agents use the same index twice.
- "rel_position" is a list of two floats indicating the desired relative position of two agents or the absolute desired position of a single agents depending if the task for two or one agent.
- "timespan" is a list of two integers represing the start and end time of the task in seconds. You should use integers approximations of what the users asks.
- "size" is a float indicating the size of the task area in meters. if not specified by the user use a default value of 1.0 meter.
- "period_num" is an integer indicating the time period index in which the task should be performed. Namely, each distincs timespan corresponds to a period. If the user specifies 3 timespans, then there are three periods. The timespan of the periods should not be overlapping, so adjust the user input to make it such.
- "operator" the operator can be only one of "always" or "eventually". if not specified use always as default.
- when multiple tasks are defined with connective AND, they will have the same period and same time span.
- When two tasks are defined with connective OR, pick one of the two and discard the other one from the list.
- When the operator mentions something like task A then task B, it means it wants the tasks to be satisfied in two subsequent period with two different timespans.  
- If time spans are not specified by the user, create new times spans of 10 seconds interval each and make sure that the start and end of two time spans are at least separated by 20 seconds.
"""


class LanguageToTasksNode(Node):

    def __init__(self):
        super().__init__('language_to_tasks_node')

        # Parameters (nice for ROS usage)
        api_key = "AIzaSyCzEL54AnzTOWlGqXp3brd-WmM2-j39gJg"

        # Gemini client
        self.client = genai.Client(api_key=api_key)

        # ROS interfaces
        self.sub = self.create_subscription(
            String,
            '/operator_command',
            self.command_callback,
            10
        )

        self.pub = self.create_publisher(
            TMsgllm,
            '/tasks',
            10
        )

        self.get_logger().info("Language → Tasks node started.")

    def command_callback(self, msg: String):
        self.get_logger().info(f"Received command: {msg.data}")

        try:
            response = self.client.models.generate_content(
                model="gemini-3-flash-preview",
                contents=SYSTEM_PROMPT + "\n\n" + msg.data,
            )

            raw_text = response.text.strip()
            self.get_logger().debug(f"Raw LLM output: {raw_text}")

            data = json.loads(raw_text)

            if "tasks" not in data:
                raise ValueError("No 'tasks' field in LLM output")

            # print row output
            self.get_logger().info(f"Parsed LLM output: {raw_text}")

            for t in data["tasks"]:
                task_msg = TMsgllm()

                task_msg.timespan    = [float(x) for x in t["timespan"]]
                task_msg.edges       = [int(x) for x in t["edges"]]
                task_msg.rel_position= [float(x) for x in t["rel_position"]]
                task_msg.size        = t['size']
                task_msg.period_num  = t["period_num"]
                task_msg.task_type   = t["operator"]

                self.pub.publish(task_msg)

                self.get_logger().info(
                    f"Published task: edge {task_msg.edges}, "
                    f"type={task_msg.task_type}, "
                    f"time=[{task_msg.timespan}]"
                )

        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse JSON from LLM output")
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LanguageToTasksNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()