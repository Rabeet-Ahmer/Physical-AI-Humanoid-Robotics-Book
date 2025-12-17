# Research: Docusaurus RAG Chat Widget

**Feature**: Docusaurus RAG Chat Widget
**Status**: Completed

## 1. Docusaurus Customization for React Components

**Unknown**: How to integrate a custom React chat widget across all Docusaurus pages.

**Findings**:
- **MDX Embedding**: Custom React components can be embedded directly in Markdown/MDX files by placing them in `src/components/` and importing them (e.g., `import MyComponent from '@site/src/components/MyComponent';`).
- **Persistent Global Components**: For components that need to appear on all pages (like a chatbot icon), modifying the Docusaurus theme layout is the standard approach. This typically involves **theme swizzling**, where a theme component (e.g., `Layout.js`) is copied from the Docusaurus theme into the local project and then customized to include the new component. This allows overriding default theme behavior.

## 2. Frontend State Persistence Across Page Navigations

**Unknown**: Best practices for maintaining chat widget state (visibility, conversation history) when navigating between Docusaurus pages.

**Findings**:
- Standard React state persistence techniques are applicable.
- **Browser Storage**: `localStorage` or `sessionStorage` are suitable for persisting data across navigations or sessions.
- **Custom Hooks**: A custom React hook (e.g., `useLocalStorage`) can be implemented to abstract the storage logic and integrate seamlessly with `useState`.
- **Global State**: For global state shared across multiple components, combining React's Context API with `localStorage` or using a state management library like Zustand with its `persist` middleware are viable options.
- **SSR Handling**: Crucially, `typeof window !== 'undefined'` checks are necessary when accessing `window.localStorage` to prevent errors during Docusaurus's server-side rendering phase.

## 3. Accessible Chat UI Component Libraries/Patterns

**Unknown**: Identifying existing accessible React chat UI patterns or libraries that align with WCAG 2.1 AA standards for quick implementation.

**Findings**:
- There isn't a single, universally adopted "accessible React chat UI library" that simplifies full WCAG 2.1 AA compliance out-of-the-box for a chat widget.
- Achieving accessibility will primarily rely on **careful custom implementation** of:
    - **ARIA roles**: Proper use of `role="log"`, `aria-live="polite"`, `aria-label`, etc., for chat history and input.
    - **Semantic HTML**: Using appropriate HTML elements for structure.
    - **Keyboard Navigation**: Ensuring all interactive elements are reachable and operable via keyboard.
    - **Focus Management**: Managing focus programmatically where necessary (e.g., when opening/closing the chat panel).

## 4. Backend API Consumption in Docusaurus/React

**Unknown**: How to make API calls to the `POST /agent` endpoint from the Docusaurus frontend, specifically regarding CORS and error handling.

**Findings**:
- **API Calls**: Standard `fetch` API or `axios` can be used to send `POST` requests to the `POST /agent` endpoint. Requests should specify `method: 'POST'`, `headers: { 'Content-Type': 'application/json' }`, and `body: JSON.stringify(data)`.
- **Error Handling**: `fetch` API responses should be checked for `response.ok` (HTTP status 200-299) to detect non-successful responses. `try...catch` blocks should be used for network errors.
- **CORS**: From the frontend perspective, if the backend API is properly configured with `CORSMiddleware` (as we configured in the backend feature), `fetch` requests will proceed without cross-origin issues. Frontend developers should be aware that CORS errors typically indicate a backend configuration problem.

## Decisions

### Decision: Implement chat widget as a custom React component integrated via Docusaurus theme swizzling.
### Rationale: Provides global presence and allows full control over rendering and state.

### Decision: Manage chat widget state using React Context API combined with `localStorage` for persistence.
### Rationale: Offers a good balance of global state management and persistence across navigations without introducing heavy external libraries.

### Decision: Build custom accessible chat UI components leveraging ARIA roles and semantic HTML.
### Rationale: Ensures WCAG compliance and tight integration with Docusaurus styling, given the lack of a suitable off-the-shelf accessible library.

### Decision: Consume backend API using the native `fetch` API.
### Rationale: Lightweight, built-in, and sufficient for the defined API interaction.
