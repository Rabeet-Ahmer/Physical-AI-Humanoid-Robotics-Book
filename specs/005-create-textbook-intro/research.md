# Research Report: Create Textbook Introduction

**Feature**: Create Textbook Introduction
**Date**: 2025-12-12
**Status**: Completed

## 1. Physical AI Definition & Scope

**Unknown**: What is the precise academic definition of Physical AI versus Generative AI?
**Findings**:
- **Generative AI (GAI)**: Focuses on synthesizing new digital content (text, images, code) using models like LLMs and GANs. It operates primarily in the digital domain.
- **Physical AI (PAI)**: Focuses on embodied intelligence where AI systems perceive, understand, and interact with the physical world through sensors and actuators. It involves a "Perception-Decision-Action" cycle.
- **Intersection**: Modern Physical AI often incorporates Generative AI (e.g., VLA - Vision-Language-Action models) for high-level reasoning and planning, while traditional control theory handles low-level execution.
- **Decision**: Define Physical AI in the introduction as "The synthesis of Large World Models (LWMs) with robotic hardware," emphasizing the "Embodied Intelligence" aspect that distinguishes it from pure software AI.

## 2. Architecture Visualization

**Unknown**: Best way to visualize the "Brain-Body-Nervous System" analogy using Mermaid.
**Findings**:
- A high-level system diagram is best.
- **Analogy Mapping**:
  - **Brain**: AI / Foundation Models (Planner, VLA)
  - **Nervous System**: ROS 2 (Middleware, Communication)
  - **Body**: Robot Hardware (Sensors, Actuators)
- **Mermaid Syntax**: Use a `graph TD` (Top-Down) or `flowchart LR` (Left-Right) to show the flow of information:
  - Brain sends *Commands* to Nervous System.
  - Nervous System sends *Signals* to Body.
  - Body sends *Feedback/Data* back up.

## 3. Module Structure & Prerequisites

**Unknown**: Confirmation of technical prerequisites.
**Findings**:
- **OS**: Ubuntu 22.04 LTS is the industry standard for ROS 2 Humble. 24.04 LTS is emerging for ROS 2 Jazzy. We will recommend **Ubuntu 22.04 LTS** as the stable baseline, mentioning 24.04 as an alternative.
- **Middleware**: **ROS 2 Humble Hawksbill** (LTS) is the most widely supported.
- **Hardware**: NVIDIA GPU is critical for modern AI/Simulation (Isaac Sim). Minimum recommendation: RTX 3060 or better.
- **Languages**: Python 3.10+ (standard in Ubuntu 22.04).

## 4. Decisions & Rationale

- **Decision**: Use the "Brain-Body-Nervous System" analogy as the central pedagogical theme.
  - **Rationale**: It provides an intuitive mental model for students connecting abstract AI concepts to concrete hardware.
- **Decision**: Structure the intro to explicitly contrast "Digital/Generative AI" with "Physical AI".
  - **Rationale**: Most students coming in 2025 will be familiar with ChatGPT but not robots. This contrast anchors new knowledge in existing knowledge.

## 5. Implementation Strategy

- Replace `docs/intro.md` content entirely.
- Use `:::info` and `:::warning` admonitions for Prerequisites to make them stand out.
- Embed the Mermaid diagram directly in the markdown.
