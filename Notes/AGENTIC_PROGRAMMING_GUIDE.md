# Agentic Programming Guide for Next Session

## What is Agentic Programming?

**Agentic Programming** = Working independently with AI as a pair programmer, where:
- **You** define the architecture and make design decisions
- **AI** implements details and explains tradeoffs
- **Together** you build production-quality systems

---

## Session Workflow (Recommended)

### Phase 1: Architecture Design (You Lead)
```
1. Read the PHASE_4_PART2_PLAN.md
2. Sketch system on paper:
   - Boxes for nodes
   - Arrows for topics
   - Labels for message types
3. Identify unclear parts
4. Ask AI: "Explain the tradeoffs between X and Y"
```

### Phase 2: Component Implementation (AI Assists)
```
1. Pick one component (start with simplest)
2. Tell AI: "Let's implement FeatureQualityMonitor"
3. AI generates code skeleton
4. You review and ask:
   - "Why did you choose this data structure?"
   - "What happens if this node crashes?"
   - "How would I test this?"
5. Iterate until you understand every line
```

### Phase 3: Integration (Collaborative)
```
1. You: "Let's connect Monitor to Selector"
2. AI: Shows topic remapping in launch file
3. You: "What if messages are out of sync?"
4. AI: Explains synchronization strategies
5. You: Make the decision, AI implements
```

### Phase 4: Testing & Debugging (You Validate)
```
1. Run the system
2. Observe behavior
3. When something breaks:
   - You: Describe what you expected vs saw
   - AI: Suggests 3 possible causes
   - You: Pick most likely, AI helps debug
4. Repeat until working
```

---

## Questions to Ask AI (Prompts That Work Well)

### Understanding Architecture
```
"Explain the data flow from Camera to RViz in this system"
"What are the failure modes of this design?"
"How would a production system handle X differently?"
```

### Code Quality
```
"Is this the idiomatic way to do X in ROS 2?"
"What error handling should I add here?"
"How would I unit test this function?"
```

### Design Decisions
```
"Should I use a service or a topic for X? Trade-offs?"
"What's the right queue size for this scenario?"
"When should I create a new node vs new class?"
```

### Debugging
```
"The monitor shows 0 features but detector shows 400. Where's the disconnect?"
"How do I trace this message through the system?"
"What debugging tools should I use for this?"
```

---

## Anti-Patterns (Don't Do This)

❌ **Copy-paste without understanding**
- Ask "Explain this line" for anything unclear

❌ **Accept first solution**
- Ask "What are 2 other ways to do this?"

❌ **Skip error handling**
- Ask "What could go wrong here?"

❌ **No testing strategy**
- Ask "How do I verify this works?"

---

## Your Role vs AI Role

### You Decide:
- System architecture (which nodes, how they connect)
- Requirements (what does "good quality" mean?)
- Priorities (performance vs clarity vs features)
- Design tradeoffs (complexity vs functionality)

### AI Helps:
- Implement your design
- Explain ROS 2 patterns
- Suggest best practices
- Debug issues
- Generate boilerplate

---

## Session Structure (3 hours)

### 0:00-0:15 - Planning
- Read plan
- Sketch architecture
- Clarify questions with AI

### 0:15-1:30 - Build Core (Monitor)
- Design message format
- Implement monitor node
- Test in isolation

### 1:30-2:15 - Build Adapter (Selector)
- Design control loop
- Implement adaptation logic
- Test with monitor

### 2:15-2:45 - Visualization
- Implement markers
- Test in RViz

### 2:45-3:00 - Integration & Demo
- Launch full system
- Test scenarios
- Document learnings

---

## Key Mindset

**Think like an architect, not a coder**:
- Start with boxes and arrows (system design)
- Then write interfaces (message definitions)
- Finally implement (with AI's help)
- Always ask "why?" before "how?"

**Iterate quickly**:
- Build simplest version first
- Test early and often
- Add complexity incrementally

**Document decisions**:
- Comment WHY not just WHAT
- Note alternatives you considered
- Explain tradeoffs you made

---

## Example Dialogue (Good Agentic Programming)

**You**: "I want the Quality Monitor to detect feature clustering. How should I approach this?"

**AI**: "Three approaches:
1. Quadtree subdivision (fast, approximate)
2. Nearest-neighbor analysis (accurate, slower)  
3. Grid histogram (simple, coarse)

Trade-offs: ..."

**You**: "Let's start with grid histogram for simplicity. I can optimize later. Show me the implementation."

**AI**: [generates code]

**You**: "Why did you use 8x8 grid? What if I want configurable?"

**AI**: "Good point! Let me add that as a parameter..."

**You**: "Perfect. Now explain how I'd unit test this function."

↑ **This is great agentic programming!** You lead, AI assists, you understand.

---

## Pre-Session Checklist

Before next session:
- [ ] Read PHASE_4_PART2_PLAN.md completely
- [ ] Sketch system diagram on paper
- [ ] List 3 questions you want answered
- [ ] Identify which component you want to build first
- [ ] Think about success criteria (how do you know it works?)

---

## Post-Session Reflection

After session:
- [ ] Can you explain the full data flow?
- [ ] Could you rebuild this from scratch?
- [ ] What design decisions did you make and why?
- [ ] What would you do differently?
- [ ] How does this connect to previous phases?

---

**Goal**: By end of session, you should understand the system well enough to:
1. Explain it to another engineer
2. Modify it to add new features
3. Debug issues independently
4. Apply these patterns to new problems

This is **real software engineering**, not just coding! 🏗️
