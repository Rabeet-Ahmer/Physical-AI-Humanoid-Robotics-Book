# Tasks: Docusaurus RAG Chat Widget

**Feature**: Docusaurus RAG Chat Widget
**Status**: Pending
**Branch**: `001-rag-chat-widget`

## Dependencies

- **US1** depends on **Phase 1** & **Phase 2** completion.
- **US2** depends on **US1** completion.
- **US3** depends on **US1** completion.

## Phase 1: Setup

**Goal**: Prepare the Docusaurus project structure for the new chat widget.

- [ ] T001 Create component directory: `src/components/ChatWidget/`
- [ ] T002 Create hooks directory: `src/hooks/`
- [ ] T003 Create CSS file for styling: `src/css/chat-widget.css`
- [ ] T004 Create API utility file: `src/utils/api.ts`

## Phase 2: Foundational (Cross-cutting components/logic)

**Goal**: Implement reusable hooks and context for state management and API calls.

- [ ] T005 Create `useLocalStorage` hook: `src/hooks/useLocalStorage.ts`
- [ ] T006 Create `ChatContext` for global state: `src/components/ChatWidget/ChatContext.tsx`
- [ ] T007 Implement `callChatApi` function in `src/utils/api.ts` for `POST /agent` calls

## Phase 3: User Story 1 - Access Chatbot

**Goal**: Implement the chatbot icon and the toggleable chat panel, integrating it into the Docusaurus layout.
**Story**: [US1] Access Chatbot (Priority: P1)

**Independent Test**:
1.  Run `npm run start` and navigate the Docusaurus site.
2.  Visually confirm the chatbot icon is fixed to the bottom-right corner.
3.  Click the icon; observe the chat UI panel appearing/disappearing without full page redraw.
4.  Navigate to different pages; confirm chat UI state (open/closed, history) persists.

**Tasks**:

- [ ] T008 [P] [US1] Create ChatIcon component: `src/components/ChatWidget/ChatIcon.tsx`
- [ ] T009 [P] [US1] Create ChatPanel container component: `src/components/ChatWidget/ChatPanel.tsx`
- [ ] T010 [US1] Swizzle Docusaurus `Layout` component: `npm run swizzle @docusaurus/theme-classic Layout -- --wrap` (if not already swizzled)
- [ ] T011 [US1] Modify `src/theme/Layout.js` (or `.tsx`) to render `ChatIcon` and `ChatPanel` (wrapped with `ChatContext.Provider`)

## Phase 4: User Story 2 - Interact with Chatbot

**Goal**: Enable users to send messages and receive responses within the chat UI.
**Story**: [US2] Interact with Chatbot (Priority: P1)

**Independent Test**:
1.  Open the chat UI.
2.  Type a question and submit it.
3.  Verify the user message, loading indicator, and chatbot response appear correctly in the history.
4.  Test with a known question and verify the response is accurate.

**Tasks**:

- [ ] T012 [P] [US2] Create ChatMessage component: `src/components/ChatWidget/ChatMessage.tsx`
- [ ] T013 [US2] Implement chat history display in `src/components/ChatWidget/ChatPanel.tsx` using `ChatMessage` components
- [ ] T014 [US2] Implement message input field, send button, and submission logic in `src/components/ChatWidget/ChatPanel.tsx`
- [ ] T015 [US2] Implement visual loading indicator for API calls in `src/components/ChatWidget/ChatPanel.tsx`
- [ ] T016 [US2] Integrate `callChatApi` from `src/utils/api.ts` for sending/receiving messages in `src/components/ChatWidget/ChatPanel.tsx`

## Phase 5: User Story 3 - Close Chatbot

**Goal**: Provide a mechanism for users to close the chat UI.
**Story**: [US3] Close Chatbot (Priority: P2)

**Independent Test**:
1.  Open the chat UI.
2.  Click the close button/icon.
3.  Verify the chat UI panel disappears, leaving only the icon.

**Tasks**:

- [ ] T017 [US3] Implement close button/icon and its functionality within `src/components/ChatWidget/ChatPanel.tsx`

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Ensure quality, accessibility, and robust error handling.

**Tasks**:

- [ ] T018 Apply styling from `src/css/chat-widget.css` to `ChatWidget` components
- [ ] T019 Implement WCAG 2.1 AA accessibility attributes (ARIA roles, keyboard nav) across all `ChatWidget` components
- [ ] T020 Verify chat state persistence across Docusaurus page navigations
- [ ] T021 Implement user-friendly error messages for API call failures in `src/components/ChatWidget/ChatPanel.tsx`
- [ ] T022 Conduct manual end-to-end testing following `quickstart.md`
