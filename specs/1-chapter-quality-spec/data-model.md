# Data Model: Chapter Quality

**Date**: 2025-12-22
**Feature**: `1-chapter-quality-spec`
**Input**: `spec.md` - Key Entities

This document defines the data structures for the entities involved in the chapter quality validation process.

## Chapter

A manuscript file containing the textbook content for a specific topic.

**Fields**:
- `file_path`: string (The path to the chapter's markdown file)
- `content`: string (The full markdown content of the chapter)
- `week_number`: integer (The week number the chapter belongs to)
- `topic_slug`: string (A URL-friendly slug for the chapter topic)

## Validation Report

A structured document detailing the pass/fail status of a chapter against each of the 8 functional requirements.

**Fields**:
- `report_id`: UUID (A unique identifier for the report)
- `chapter_file_path`: string (The path to the chapter that was validated)
- `validation_timestamp`: datetime (When the validation was run)
- `overall_status`: string (Enum: "PASS", "FAIL")
- `agent_results`: array (An array of objects, one for each agent validation)

### Agent Result

**Fields**:
- `agent_name`: string (The name of the pedagogical agent, e.g., "Developmental Staging Agent")
- `status`: string (Enum: "PASS", "FAIL")
- `score`: float (The numerical score from the agent, e.g., 8.5)
- `feedback`: string (Detailed feedback from the agent explaining the result)
- `requirement_violated`: string (The functional requirement that was violated, e.g., "FR-001")
