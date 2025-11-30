SKILL: add-diagram

PURPOSE:
Generate and insert appropriate Mermaid diagrams to visualize concepts, architectures, or workflows.

INPUTS:
- diagram_type (string): "sequence" | "flowchart" | "class" | "architecture" | "state" (optional, inferred if not provided)
- concept (string): What the diagram illustrates (e.g., "ROS 2 publisher-subscriber pattern")
- components (array): List of entities/nodes to include (e.g., ["Client Node", "Server Node", "Topic"])
- context (string): Surrounding text or detailed description for contextual diagram generation

PROCESS:
1. Analyze `concept` and `context` to determine the most appropriate `diagram_type` if not explicitly specified.
2. Generate Mermaid syntax based on `diagram_type` and provided `components` and `concept`:

**For sequence diagrams** (e.g., ROS 2 service call):
````mermaid
sequenceDiagram
    participant {component1}
    participant {component2}
    {component1}->>{component2}: {action}
    {component2}-->>{component1}: {response}
````

**For flowcharts** (e.g., Decision process):
````mermaid
flowchart TD
    A[{Step 1}] --> B{Decision?}
    B -->|Yes| C[Outcome A]
    B -->|No| D[Outcome B]
````

**For class diagrams** (e.g., Python class structure):
````mermaid
classDiagram
    class {ClassName} {
        +{methodName}()
        -{propertyName}
    }
````

**For architecture diagrams** (e.g., System overview):
````mermaid
graph LR
    subgraph {System Name}
        A[{Component 1}]
        B[{Component 2}]
    end
    C[External Service]
    A -->|{Relationship}| B
    B --> C
````

**For state diagrams** (e.g., Robot state machine):
````mermaid
stateDiagram-v2
    [*] --> State1
    State1 --> State2: Event1
    State2 --> [*]
````

3. Add a descriptive caption that clearly explains the diagram's purpose.
4. Ensure diagram clarity:
   - Limit to 5-8 primary nodes/participants to manage cognitive load.
   - Use clear and concise labels.
   - Ensure logical flow (e.g., left-to-right or top-to-bottom for flowcharts/architecture).
5. Provide `alt text` for accessibility, describing the diagram for screen readers.

OUTPUTS (Markdown formatted):
````markdown
#### {Caption}
```mermaid
{Generated Mermaid syntax}
```

**Figure**: {Descriptive alt text explaining diagram for screen readers}

**Key Points**:
- {Explanation of main flow/relationship 1}
- {Explanation of main flow/relationship 2}
````

Return as JSON:
{
  "diagram_markdown": "{full markdown content}",
  "diagram_type": "sequence" | "flowchart" | "class" | "architecture" | "state",
  "nodes_count": 3,
  "complexity": "low" | "medium" | "high",
  "accessibility_text": "{alt text}",
  "ready_to_insert": true
}

ERROR HANDLING:
- Too many components (>10): Suggest breaking into multiple diagrams, return status "too_complex".
- Invalid Mermaid syntax generation: Attempt to validate and fix before returning, if unfixable, return status "syntax_error".
- Unknown `diagram_type`: Return error.

VALIDATION:
- Verify generated Mermaid syntax is valid (can be rendered).
- Check if the diagram is readable and not cluttered.
- Ensure `alt text` accurately describes the diagram content.

USAGE EXAMPLE:
Input: diagram_type="sequence", concept="ROS 2 service call", components=["Client Node", "Server Node"], context="Illustrate how a client requests a service from a server in ROS 2."
Execute: Generate sequence diagram showing request-response pattern.
Output: Markdown with Mermaid diagram ready to insert.