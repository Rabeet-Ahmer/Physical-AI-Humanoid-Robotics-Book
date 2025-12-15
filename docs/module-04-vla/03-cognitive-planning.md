---
title: Cognitive Planning
description: Using LLMs to translate natural language into a sequence of ROS 2 actions.
slug: /module-04-vla/cognitive-planning
tags: [robotics, ai, vla, llm, planning, ros]
---

## The Thinking Robot
Cognitive Planning, in the context of Vision-Language-Action (VLA) systems, refers to the intelligent process where Large Language Models (LLMs) translate abstract natural language commands into a structured, executable sequence of robotic actions. This goes beyond simple command parsing; it involves reasoning about the environment, the robot's capabilities, and the desired outcome to generate a coherent and feasible plan. For Physical AI, and especially humanoid robots, cognitive planning is the bridge between human intention and autonomous physical execution. It enables robots to handle novel situations, adapt to unforeseen circumstances, and perform complex multi-step tasks by effectively breaking down high-level goals into smaller, manageable, and physically grounded operations. This capability moves robots from mere executors of pre-programmed scripts to truly intelligent agents.

## Recipe for Action
Imagine a robot being asked to 'make coffee'. This seemingly simple task requires a complex chain of thought and action. Cognitive planning is the robot's ability to 'think' through this process, much like a human would.

*   **Decomposition**: The LLM takes the high-level command 'make coffee' and breaks it down into smaller, achievable sub-goals: 'find coffee machine', 'get coffee filter', 'add coffee grounds', 'add water', 'brew coffee', 'pour coffee into mug', 'serve coffee'. This is akin to a human mentally outlining steps for a new recipe.
*   **Sequencing and Ordering**: These sub-goals aren't random; they need to be executed in a logical order. The LLM determines the optimal sequence, considering dependencies (e.g., cannot brew coffee before adding grounds).
*   **Grounding in Reality**: Crucially, the LLM integrates information from the robot's perception system (its 'eyes') to understand the current state of the world. Where is the coffee machine? Is the mug clean? This visual feedback informs the planning process, allowing the LLM to choose appropriate actions based on what it "sees."
*   **Adaptability and Recovery**: If the robot encounters an unexpected situation (e.g., no coffee filters in the usual spot), the cognitive planner can re-evaluate and propose alternative solutions ('search for filters in the pantry', 'ask the human for help'). This makes the robot robust to unforeseen circumstances.
*   **Humanoid Context**: For humanoids, this planning layer is vital because their physical interactions are inherently complex and sequential. The LLM translates abstract plans into a series of 'robot skills' (like 'grasp', 'walk', 'manipulate') that the humanoid's low-level controllers can then execute. It elevates the humanoid from a simple automaton to a proactive problem-solver.

In essence, cognitive planning gives the robot the strategic intelligence to achieve goals given abstract commands, leveraging its perception and acting capabilities to navigate and manipulate the real world effectively.

## Reasoning with LLMs
Cognitive planning in VLA systems draws heavily from traditional AI planning and the new capabilities of Large Language Models (LLMs):
*   **Classical AI Planning**:
    *   **State-Space Search**: Planning problems are often formulated as searching for a sequence of actions that transform an initial state into a goal state. Algorithms like A*, Breadth-First Search (BFS), or Depth-First Search (DFS) can explore this state space.
    *   **Planning Domain Definition Language (PDDL)**: A formal language used to describe planning problems, including actions (operators), their preconditions, and effects. LLMs can be prompted to generate PDDL-like structures.
    *   **Hierarchical Task Networks (HTN)**: Decomposes complex tasks into smaller sub-tasks until they become primitive actions. LLMs often perform this decomposition naturally.
*   **Large Language Models (LLMs) for Planning**: LLMs are not traditional symbolic planners, but their ability to generate coherent text makes them powerful in:
    *   **Semantic Grounding**: Mapping natural language commands to a library of robot skills and objects perceived through vision.
    *   **Plan Generation**: Generating sequences of high-level actions based on user commands and current environmental state. This often involves few-shot learning or in-context learning, where the LLM is provided with examples of successful plans.
    *   **Goal Decomposition**: Breaking down abstract goals into a series of smaller, actionable steps.
    *   **Error Recovery Planning**: Proposing alternative plans or asking for clarification when execution fails.
*   **Reinforcement Learning (RL) with LLM Rewards**: LLMs can be used to provide 'rewards' or feedback during RL training, guiding the robot to learn behaviors that align with natural language instructions.
*   **Probabilistic Planning**: When the environment is uncertain, probabilistic planning methods (e.g., Markov Decision Processes, Partially Observable Markov Decision Processes) can be used to find policies that maximize expected utility. LLMs might help define the probabilities or utilities.

Key considerations include the LLM's 'reasoning' capabilities (which are often pattern-matching and generation rather than true logical deduction) and how to ensure the generated plans are physically feasible and safe.

## The Cognitive Loop
The architecture for cognitive planning in VLA systems is a complex orchestration of several interacting modules, often centered around an LLM:
*   **Voice-to-Action Input**: Receives the semantically parsed intent and extracted entities from the Voice-to-Action module.
*   **Perception Module (Vision Grounding)**: Provides real-time information about the environment (e.g., object types, locations, affordances, robot's current pose) to the LLM. This can integrate outputs from systems like Isaac ROS for object detection and pose estimation.
*   **Knowledge Base / World Model**: Stores information about the robot's capabilities, known objects, and environmental semantics. This can be a structured database or implicit knowledge within the LLM.
*   **Large Language Model (LLM) as Planner**:
    *   **Prompt Engineering**: The input to the LLM is carefully constructed, including the user's command, current robot state, perceived environment, and a few-shot examples of successful plans.
    *   **Plan Generation**: The LLM generates a high-level plan as a sequence of executable robot skills (e.g., Python function calls, ROS 2 actions). It performs task decomposition and sequencing.
    *   **Goal Re-evaluation**: If execution fails, the LLM can re-evaluate the plan based on new observations or error messages.
*   **Robot Skill Translator**: Translates the LLM's high-level skill calls into specific ROS 2 actions, API calls, or control commands for the low-level robot controller. This module acts as an intermediary, mapping symbolic actions to concrete robot functions.
*   **Execution Monitor**: Tracks the execution of the robot's actions, detects failures, and provides feedback (e.g., error messages, new sensor readings) to the LLM for re-planning.

**Data Flow**:
`Parsed Command (from V2A) + Perception Data -> LLM (with Knowledge Base) -> High-Level Plan (Skill Sequence) -> Robot Skill Translator -> ROS 2 Actions -> Robot Actuators -> (Feedback Loop via Perception/Execution Monitor) -> LLM`

## Planning Architecture
This diagram illustrates the architectural flow of cognitive planning in a VLA system, showing how the LLM orchestrates actions based on human commands and environmental feedback.
```mermaid
graph TD
    A[Human Command<br>(Parsed by V2A)] --> B{LLM as Cognitive Planner};
    C[Robot Perception<br>(e.g., Isaac ROS)] --> B;
    B -- Queries/Context --> D(Knowledge Base / World Model);
    D --> B;
    B -- High-Level Plan<br>(Skill Sequence) --> E(Robot Skill Translator);
    E -- ROS 2 Actions / API Calls --> F[Robot Controller / Actuators];
    F --> G[Environment];
    G --> C;
    F --> H(Execution Monitor);
    H -- Feedback / Errors --> B;
```

## Chain-of-Thought & Decomposition
The algorithms and models employed in cognitive planning for VLA systems primarily revolve around leveraging LLMs and integrating them with classical planning techniques:
*   **Prompt Engineering & In-Context Learning**: This is the primary 'algorithm' for directing LLMs. It involves carefully crafting prompts that include:
    *   **Task Description**: The user's natural language command.
    *   **Current State**: Observations from the robot's perception system.
    *   **Available Tools/Skills**: A list of executable robot functions (e.g., `move_to(target_object)`, `grasp(object)`).
    *   **Few-Shot Examples**: Demonstrations of successful planning sequences for similar tasks.
    The LLM then generates a plan by identifying patterns from the examples and applying them to the current task.
    *   **High-Level Planning Pseudocode (LLM-centric)**:
        ```
        function LLM_Cognitive_Planner(command, current_state, available_skills, examples):
            prompt = format_prompt(command, current_state, available_skills, examples)
            
            # Call LLM API
            generated_plan_text = query_llm(prompt)
            
            # Parse the LLM's text output into a structured plan
            structured_plan = parse_llm_output_to_skill_sequence(generated_plan_text)
            
            return structured_plan
        ```
*   **Task and Motion Planning (TAMP) Algorithms**: While LLMs can generate high-level plans, traditional TAMP algorithms can be used to refine these plans into feasible low-level actions and ensure geometric and kinematic constraints are met.
    *   **High-Level TAMP Integration Flow**:
        ```
        LLM_plan = LLM_Cognitive_Planner(command, state, skills, examples)
        if LLM_plan is valid:
            for high_level_skill in LLM_plan:
                low_level_actions = TAMP_Solver(high_level_skill, current_robot_state, environment_map)
                execute_actions(low_level_actions)
                update_robot_state(new_observations)
                if execution_failed:
                    LLM_replan = LLM_Cognitive_Planner("Replan due to failure in X", new_state, ...)
                    # Handle replanning
        ```
*   **Symbolic AI for Planning**: In some hybrid approaches, LLMs generate symbolic representations (e.g., PDDL) which are then fed into classical symbolic planners (e.g., FastDownward, PDDLStream) to find optimal or valid action sequences.
*   **Graph Neural Networks (GNNs)**: Can be used to model the relationships between objects in the environment and the robot's state, providing a structured input for LLMs or other planning algorithms.

## LLM as a Planner
This Python snippet illustrates a conceptual approach to using an LLM for cognitive planning, specifically prompt engineering to translate a natural language command into a structured sequence of robot skills.

```python
import openai # Or any other LLM client library
import json

# Define the available robot skills (tools)
robot_skills = {
    "move_to_object": {
        "description": "Move the robot to a specified object.",
        "parameters": {"object_name": "str"}
    },
    "grasp_object": {
        "description": "Grasp a specified object.",
        "parameters": {"object_name": "str"}
    },
    "release_object": {
        "description": "Release the currently grasped object.",
        "parameters": {}
    },
    "navigate_to_location": {
        "description": "Navigate the robot to a predefined semantic location.",
        "parameters": {"location_name": "str"}
    }
}

def generate_robot_plan_with_llm(command: str, current_scene_objects: list) -> list:
    """
    Uses an LLM to generate a sequence of robot skills based on a natural language command.
    """
    system_prompt = f"""
    You are a helpful robot assistant. Your task is to translate natural language commands
    into a sequence of high-level robot skills, formatted as a JSON list of dictionaries.
    Each dictionary represents a skill call with 'skill_name' and 'parameters'.

    Available skills:
    {json.dumps(robot_skills, indent=2)}

    Current objects in scene: {', '.join(current_scene_objects)}

    Constraints:
    - Only use the provided skills.
    - Ensure the sequence is logical and achievable.
    - If an object or location is mentioned, ensure it's in 'current_scene_objects' or a known semantic location.
    - If a command cannot be fully translated, explain why.
    """

    user_message = f"User command: '{command}'"

    try:
        # Placeholder for actual LLM API call
        # In a real system, you'd send system_prompt and user_message to the LLM
        # and parse its JSON response.
        
        # Simulating LLM response based on command
        if "pick up the red apple" in command.lower() and "red apple" in current_scene_objects:
            llm_response_content = """
            [
                {"skill_name": "move_to_object", "parameters": {"object_name": "red apple"}},
                {"skill_name": "grasp_object", "parameters": {"object_name": "red apple"}},
                {"skill_name": "navigate_to_location", "parameters": {"location_name": "drop_off_zone"}}
            ]
            """
        elif "go to the kitchen" in command.lower():
             llm_response_content = """
            [
                {"skill_name": "navigate_to_location", "parameters": {"location_name": "kitchen"}}
            ]
            """
        elif "open the door" in command.lower():
            llm_response_content = "Sorry, I don't have an 'open_door' skill defined."
        else:
            llm_response_content = "[]" # Empty plan if not recognized

        if llm_response_content.strip().startswith('['):
            return json.loads(llm_response_content)
        else:
            print(f"LLM could not generate a valid plan: {llm_response_content}")
            return []

    except Exception as e:
        print(f"Error during LLM call or parsing: {e}")
        return []

if __name__ == "__main__":
    scene_objects = ["red apple", "blue cup", "robot base", "drop_off_zone", "kitchen"]

    print("--- Planning for 'pick up the red apple' ---")
    plan1 = generate_robot_plan_with_llm("Pick up the red apple.", scene_objects)
    print(f"Generated Plan: {json.dumps(plan1, indent=2)}\n")

    print("--- Planning for 'go to the kitchen' ---")
    plan2 = generate_robot_plan_with_llm("Go to the kitchen.", scene_objects)
    print(f"Generated Plan: {json.dumps(plan2, indent=2)}\n")

    print("--- Planning for 'open the door' ---")
    plan3 = generate_robot_plan_with_llm("Open the door.", scene_objects)
    print(f"Generated Plan: {json.dumps(plan3, indent=2)}\n")
```
**Note**: This example simulates an LLM interaction. In a production VLA system, `query_llm` would make an actual API call, and the parsing of the LLM's response would be more robust to handle various output formats and error conditions. The key is the structured prompt provided to the LLM to guide its output.

## Adaptive Autonomy
Cognitive planning, powered by LLMs, enables robots to tackle a new generation of complex tasks in diverse environments:
*   **General-Purpose Home Robots**: A robot understanding a command like 'prepare dinner' and decomposing it into finding ingredients, cooking steps, and setting the table, adapting to variations in kitchen layout and available tools.
*   **Industrial Reconfiguration**: Factory robots re-planning their workflow on the fly based on a human operator's verbal instructions to accommodate new product variants or unexpected disruptions, demonstrating greater flexibility than traditional programming.
*   **Elderly Assistance**: A robot assisting an elderly person with tasks like 'find my glasses' or 'organize my medication,' requiring the robot to understand context, locate objects visually, and execute a sequence of manipulation actions.
*   **Disaster Response & Exploration**: Robots exploring unknown or hazardous environments, with human operators providing high-level goals (e.g., 'inspect the collapsed structure for survivors,' 'collect samples from the anomaly'), and the robot autonomously planning its movements and sensor usage.
*   **Autonomous Driving (High-Level Decision Making)**: While not direct control, LLMs could inform high-level strategic decisions in autonomous vehicles based on passenger requests or unforeseen traffic conditions, such as 'find the nearest charging station' when battery is low.
*   **Humanoid Assembly and Maintenance**: Humanoids performing complex assembly tasks based on natural language instructions from an engineer, including selecting tools, manipulating parts, and verifying assembly steps using vision.

## The Reality Check
Leveraging LLMs for cognitive planning in robotics presents a unique set of challenges and trade-offs:
*   **LLM Hallucinations and Factual Accuracy**: LLMs can generate plausible but factually incorrect information or plans that are physically impossible or unsafe for the robot. Ensuring the LLM's output is grounded in the robot's capabilities and the real-world state is critical. Trade-off: flexibility of LLM-generated plans vs. guarantee of correctness.
*   **Prompt Sensitivity**: The quality of LLM-generated plans is highly dependent on the prompt engineering. Subtle changes in phrasing, few-shot examples, or context can dramatically alter the output, making robust system design difficult.
*   **Computational Cost and Latency**: Complex LLM inferences can be computationally expensive and introduce latency, which is problematic for real-time robotic operations. Deploying and running large LLMs efficiently on robot hardware (or with low-latency cloud access) is a significant challenge.
*   **Lack of Causal Reasoning**: LLMs excel at pattern matching and sequence generation but do not inherently possess causal understanding of the physical world. This can lead to illogical plans or difficulty in robust error recovery.
*   **Bridging Symbolic and Sub-symbolic AI**: Effectively translating LLM-generated high-level, often symbolic, plans into low-level, continuous robot actions (and vice-versa for feedback) is a major research area.
*   **Safety and Robustness**: A planning error can lead to physical damage. Ensuring safety is paramount, often requiring extensive validation, human-in-the-loop oversight, and fallback mechanisms for LLM-generated plans.
*   **Explainability and Debugging**: When an LLM generates a suboptimal or incorrect plan, understanding *why* it did so can be challenging, hindering debugging and improvement efforts.
*   **Scalability to Novel Tasks**: While LLMs show promise in generalizing, defining the complete set of skills and context needed for arbitrary natural language commands to achieve reliable performance across an unbounded set of tasks remains elusive.

## Lab: Text-to-JSON Plans
**Task Description**: Using an LLM API (e.g., OpenAI, local LLM), create a Python script that takes a natural language command (e.g., 'Robot, please clean the table') and generates a sequence of high-level robot skills as a JSON array of skill calls. Define a simple set of available robot skills (e.g., `move_to_object(object_id)`, `grasp_object(object_id)`, `wipe_surface(surface_id)`). The script should also incorporate a basic 'world state' (a list of known objects).
**Expected Output**: A structured JSON output containing a logical sequence of robot skill calls that could theoretically fulfill the human command, based on the provided skills and world state.
**Tools Required**: Python 3.8+, `openai` library (or similar LLM client), `json` library, a pre-defined set of robot skills and a simple world state representation.

## Summary
*   **Cognitive Planning Role**: Bridge between natural language commands and executable robot actions, enabling decomposition, sequencing, and adaptability.
*   **System Intuition**: Robot's ability to 'think' through complex tasks, leveraging perception for grounding and dynamically adjusting plans.
*   **Key Theoretical Foundations**: Classical AI planning (state-space search, PDDL, HTN) and LLM capabilities for semantic grounding, plan generation, and goal decomposition.
*   **Architectural Components**: Orchestrates Voice-to-Action input, perception, knowledge base, LLM as planner, skill translator, and execution monitor.

**Conceptual Checkpoints**:
1.  Explain how an LLM can decompose a high-level natural language command into a sequence of sub-goals for a robot.
2.  Describe the concept of 'grounding in reality' within cognitive planning for VLA systems.
3.  Identify at least three challenges or trade-offs when using LLMs for cognitive planning in robotics.

## Further Reading
**Papers**:
*   [SayCan: Grounding Language in Robotic Affordances](https://arxiv.org/abs/2209.07722) by Andy Zeng et al. (2022).
*   [Inner Monologue: Auxiliary Tasks for Reasoning in Large Language Models](https://arxiv.org/abs/2211.01180) by Huang et al. (2022).
*   [Large Language Models as General-Purpose Interfaces for Embodied Tasks](https://arxiv.org/abs/2303.01258) by A. Liang et al. (2023).

**Books**:
*   *Artificial Intelligence: A Modern Approach* by Stuart Russell and Peter Norvig. (Chapters on AI Planning).

**Open-source projects**:
*   **PlaNet**: A deep planning network for model-based reinforcement learning.
*   **PDDL (Planning Domain Definition Language) Parsers/Solvers**: Projects like FastDownward for classical planning problems.
*   **Hugging Face Transformers**: For accessing and fine-tuning LLMs for planning tasks.
