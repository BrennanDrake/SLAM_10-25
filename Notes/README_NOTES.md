# Documentation Structure Guide

## Current Documentation Organization

### 🧑‍💻 User-Facing Documentation
**Purpose**: Help humans understand and work with the project

1. **PROJECT_DESIGN.md** - High-level architecture and roadmap
2. **DEVELOPMENT_LOG.md** - Human-readable progress and decisions  
3. **TROUBLESHOOTING_GUIDE.md** - Practical solutions to common problems
4. **ALGORITHM_NOTES.md** - Technical details for researchers/developers
5. **CHANGELOG.md** - Version history and milestones

### 🤖 Model-Optimized Documentation
**Purpose**: Provide context for AI assistants in future sessions

1. **cascade_prompt.txt** - Direct instructions for AI context
2. **Memory entries** - Structured key learnings stored via create_memory tool

## Documentation Philosophy

### Why Current Format Works Well

The current documentation serves **both purposes** effectively because:

1. **Structured Markdown** is easily parsed by both humans and AI
2. **Clear headings** enable quick navigation and context understanding
3. **Code examples** are formatted consistently
4. **Technical details** are explained with context

### When to Consider Separation

You might want separate model-interpretable docs when:

1. **Dealing with complex state** that needs JSON/YAML structure
2. **Tracking numerous small details** that would clutter human docs
3. **Optimizing token usage** for AI context windows
4. **Storing metadata** that humans don't need to see

## Recommended Approach

### Keep Unified Documentation (Current Approach) ✅
**Advantages**:
- Single source of truth
- No duplication of effort
- Both audiences benefit from complete context
- Easier to maintain

**Best Practices**:
```markdown
## Section Title
[Human-friendly explanation]

### Technical Details
[Structured information that AI can parse]

### Implementation
```code
[Actual code that both can reference]
```
```

### Optional: Add AI Hints
If needed, you can add AI-specific hints without separate files:

```markdown
<!-- AI_CONTEXT: This section contains critical build fixes -->
## Build Issues

<!-- AI_MEMORY: TF2 headers require double directory path in Jazzy -->
### TF2 Include Paths
```

## Timestamp Strategy Recommendation

### Approach 1: Rolling Updates (Recommended) ✅
```markdown
## Section Title
*Last updated: 2025-09-26*
[Content]

### Subsection
*Added: 2025-09-26*
[New content]
```

### Approach 2: Version Tracking
```markdown
## Section Title [v1.2]
[Content]

### Change History
- v1.2 (2025-09-26): Added TF2 workaround
- v1.1 (2025-09-25): Initial implementation
```

### Approach 3: Git-Based
- Rely on git history for detailed tracking
- Only timestamp major milestones in docs
- Use `git blame` for line-by-line history

## Maintenance Guidelines

### Update Frequency
- **CHANGELOG.md**: After each significant milestone
- **TROUBLESHOOTING_GUIDE.md**: As issues are discovered
- **DEVELOPMENT_LOG.md**: After each work session
- **ALGORITHM_NOTES.md**: When implementing new algorithms
- **Memories**: For critical learnings that affect future development

### What to Document
- **Always**: Breaking changes, new algorithms, critical fixes
- **Usually**: Parameter tuning results, performance improvements
- **Sometimes**: Minor refactoring, documentation updates
- **Rarely**: Cosmetic changes, comment updates

---
*Documentation guide created: 2025-09-26*
