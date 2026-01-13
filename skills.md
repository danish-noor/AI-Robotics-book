# Agent Skills Library

These skills are reusable, composable capabilities used by Claude Code subagents.

---

## 📚 Skill 1: Spec-to-Chapter Transformation

**Description**
Convert formal specifications into structured educational chapters.

**Used By**
- Book Orchestrator Agent
- Curriculum & Content Agent

**Steps**
1. Parse learning objectives
2. Identify prerequisite knowledge
3. Generate section outline
4. Write content
5. Validate against spec

---

## 🧩 Skill 2: Concept Chunking for RAG

**Description**
Chunk educational content for semantic retrieval.

**Used By**
- RAG & Chatbot Agent

**Rules**
- 300–500 tokens per chunk
- Preserve semantic boundaries
- Attach metadata:
  - module
  - chapter
  - difficulty
  - hardware dependency

---

## 🔍 Skill 3: Context-Restricted Answering

**Description**
Force the model to answer strictly from selected text.

**Used By**
- RAG & Chatbot Agent

**Implementation**
- Inject selection as system constraint
- Disable global retrieval
- Enforce citation-only responses

---

## 🧠 Skill 4: User Background Inference

**Description**
Adapt content based on user profile.

**Used By**
- Auth & Personalization Agent

**Signals**
- Hardware availability
- Programming experience
- Robotics exposure

**Outputs**
- Simplified explanations
- Advanced callouts
- Hardware-specific warnings

---

## 🌐 Skill 5: Technical Translation (English → Urdu)

**Description**
Translate content while preserving technical accuracy.

**Rules**
- Do not translate:
  - ROS 2
  - Gazebo
  - Isaac
  - URDF
- Maintain glossary consistency
- Prefer explanatory Urdu over literal translation

---

## 🎯 Skill 6: Bonus Optimization Mapping

**Description**
Continuously map features to scoring criteria.

**Used By**
- Evaluation & QA Agent

**Outputs**
- Feature-to-points matrix
- Missing bonus alerts

---

## 🧪 Skill 7: Sim-to-Real Validation

**Description**
Ensure simulated workflows realistically transfer to hardware.

**Used By**
- Robotics Systems Agent

**Checks**
- Jetson memory constraints
- Sensor compatibility
- Latency risks

---

## 🎥 Skill 8: Demo Compression (90s Rule)

**Description**
Condense project demo into a 90-second narrative.

**Used By**
- Evaluation & QA Agent

**Structure**
1. Problem (10s)
2. Book + AI (25s)
3. Chatbot (25s)
4. Personalization + Urdu (20s)
5. Closing (10s)
