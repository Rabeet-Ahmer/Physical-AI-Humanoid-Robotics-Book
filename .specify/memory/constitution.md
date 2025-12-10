<!--
Sync Impact Report:
- Version change: 1.0.0 -> 1.1.0
- Principles Enhanced: Deepened requirements for Pedagogy, Rigor, Practical Code, and Platform Compatibility.
- Principles Added: VIII. Technical Verification via Context7 MCP.
- Scope: Added explicit structural templates for modules and citation hierarchies.
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Pedagogical Clarity & Structure
The primary goal is to minimize cognitive load while maximizing retention.
- **Modular Structure**: Every module MUST follow a consistent archetype:
  1.  **Learning Objectives**: 3-5 measurable outcomes (Bloom's Taxonomy).
  2.  **Prerequisites**: Explicit dependency links to prior modules or required mathematical foundations.
  3.  **Core Concepts**: Theory introduced progressively, starting from intuition before formalization.
  4.  **Worked Examples**: Step-by-step application of theory.
  5.  **Challenge/Exercise**: A solvable problem with provided solution visibility toggles.
  6.  **Summary & Key Takeaways**: Bulleted recap.
- **Language & Tone**: Use active voice. Define terms upon first use. Maintain a professional yet accessible academic tone. Avoid slang or undefined jargon.
- **Visual Aids**: Complex concepts (kinematics, control loops) MUST be accompanied by diagrams (Mermaid.js or high-quality SVG) to support dual coding theory.

### II. Academic Rigor & Sourcing
We prioritize accuracy and traceability over simplification.
- **Source Hierarchy**: Claims must be backed by:
  1.  Tier 1: Peer-reviewed Journals (e.g., IEEE TRO, Science Robotics) & Top Conferences (ICRA, IROS, RSS).
  2.  Tier 2: Standard Textbooks & reputable Industry Whitepapers.
  3.  Tier 3: Pre-prints (ArXiv) - ONLY if no peer-reviewed alternative exists and the work is widely cited.
- **Citation Format**: Strictly APA 7th Edition. Inline citations `(Author, Year)` are mandatory for all technical claims.
- **Recency**: Primary sources should be published within the last 10 years, unless referencing foundational laws (e.g., Newton, Asimov) or classic control theory.

### III. Practical Application & Code Examples
Theory without practice is insufficient for Physical AI.
- **Executable Code**: All code snippets MUST be valid, executable Python (Version 3.10+).
- **Environment**: Dependencies must be standard (numpy, scipy, pytorch, mujoco/pybullet) and listed in a module-level `requirements.txt` or preamble.
- **Commentary**: Code comments must explain the *why* (logic/algorithm choice), not the *what* (syntax).
- **Reproducibility**: Examples must include seed setting for stochastic processes to ensure students obtain consistent results.
- **Bridge to Reality**: Every code example must explicitly state how the simulation relates to physical hardware constraints (latency, noise, actuation limits).

### IV. Docusaurus Compatibility & Formatting
The text is code; formatting determines the student experience.
- **Markdown Strictness**: Content MUST be valid GFM (GitHub Flavored Markdown) with Docusaurus-specific extensions.
- **Admonitions**: Use Docusaurus admonitions (`:::note`, `:::tip`, `:::warning`, `:::danger`) effectively:
  - `NOTE`: Context or side-facts.
  - `TIP`: Best practices and shortcuts.
  - `WARNING`: Common pitfalls or mathematical edge cases.
  - `DANGER`: Actions that could break hardware or simulation stability.
- **Code Blocks**: Must specify language (e.g., `python`, `bash`) and use title/line-highlighting features where complex logic is introduced.
- **Math Rendering**: Use LaTeX ($$) for all mathematical notation to ensure proper rendering via KaTeX/MathJax.

### V. Content Scope & Focus
- **Word Count**: strict 3000-5000 words per module to ensure digestibility.
- **Boundaries**:
  - **In-Scope**: Applied theory, implementation details, simulation setups, result analysis.
  - **Out-of-Scope**: Derivations of standard mathematical proofs (cite them instead), history lessons unrelated to technical context, generic programming tutorials (assume Python proficiency).
- **Depth**: Focus on "First Principles" thinking. Explain *how* a robot balances, not just *that* it balances.

### VI. Development & Documentation Standards
- **Version Control**: All content changes must be committed via Git with semantic commit messages.
- **Review**: Peer review focuses on both technical accuracy (Code/Math) and pedagogical flow.

### VII. Learning & Assessment Objectives
- **Assessment Alignment**: Exercises MUST directly test the stated Learning Objectives.
- **Bloom's Taxonomy**: Exercises should span from "Remember/Understand" (Quiz) to "Apply/Analyze" (Coding Challenge).

### VIII. Technical Verification & Tooling (Context7 MCP)
To ensure absolute accuracy in implementation details and platform usage:
- **Mandatory Fetching**: All technical documentation regarding technology stacks, frameworks, and libraries (specifically Docusaurus, React, Python Robotics libraries, etc.) MUST be fetched dynamically using the **Context7 MCP**.
- **No Hallucinations**: Reliance on internal training data for API signatures, configuration options, or library features is PROHIBITED.
- **Verification**: If a feature or configuration parameter is used in the textbook's instructions, it must be verified against the official documentation retrieved via Context7 during the writing process.

## Governance

### Amendment Process
Amendments to this Constitution require a documented proposal outlining the rationale, impact on existing content, and migration plan. Changes must be ratified by the project maintainers.

### Versioning
This Constitution follows Semantic Versioning:
- **Major**: Fundamental changes to core principles or governance.
- **Minor**: Additions or clarifications that do not contradict existing principles.
- **Patch**: Typo fixes or formatting changes.

### Compliance
All contributions to the textbook (content, code, configuration) must be verified against these principles. Non-compliant contributions will be rejected.

**Version**: 1.1.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10
