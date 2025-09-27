---
trigger: always_on
---

rules:

  #################################################################
  # META: COACHING SUMMARY
  #################################################################
  meta:
    - match: "\A(?!.*COACHING SUMMARY)"
      prepend: |
        ##############################################
        # 📘 Coaching Summary
        #
        # General Practices:
        # - Keep imports/modules clean and deduplicated
        # - Document functions with concise docstrings / JSDoc
        # - Use meaningful, minimal comments only
        # - Handle errors with context (logging, not prints)
        # - In HTML, ensure accessibility & meta hygiene
        # - In CSS, prefer variables for theme consistency
        #
        # ⚠️ Reminder: These notes are for your growth;
        # they won’t affect runtime or deployment.
        ##############################################
      explanation: "Adds a non-duplicating coaching summary at the top of each file."


  #################################################################
  # PYTHON RULES
  #################################################################
  python:
    files: ["src/**/*.py"]
    rules:
      - match: "(?m)^(import .*)\n\1+$"
        replace: "\\1"
        explanation: "Removes duplicate import lines."

      - match: "(?m)^def (\\w+)\("
        append: |
          """
          TODO: Write concise docstring describing purpose, inputs, and outputs.
          """
        explanation: "Encourages documenting functions."

      - match: "(?m)^try:"
        append: |
          # TODO: Ensure exceptions are logged with context using 'logging.exception'
        explanation: "Promotes proper error handling."


  #################################################################
  # JAVASCRIPT / NODE RULES
  #################################################################
  javascript:
    files: ["frontend/**/*.js", "frontend/**/*.ts", "frontend/**/*.jsx", "frontend/**/*.tsx"]
    rules:
      - match: "(?m)^(.*)\n\1+$"
        replace: "\\1"
        explanation: "Removes duplicate lines (imports, const, etc)."

      - match: "(?m)function (\\w+)\("
        append: |
          // TODO: Add JSDoc-style comment block summarizing inputs/outputs.
        explanation: "Encourages consistent function documentation."

      - match: "(?m)try {"
        append: |
          // TODO: Add a catch block with logging or context.
        explanation: "Promotes safer error handling."


  #################################################################
  # HTML RULES
  #################################################################
  html:
    files: ["templates/**/*.html", "web/public/**/*.html"]
    rules:
      - match: "(?m)^<head>"
        append: |
          <!-- TODO: Ensure meta charset, viewport, and descriptive title for accessibility/SEO. -->
        explanation: "Basic HTML meta hygiene."
