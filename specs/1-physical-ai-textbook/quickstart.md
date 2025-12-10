# Developer Quickstart

## Prerequisites

*   **Node.js**: Version 18.0 or higher.
*   **Git**: For version control.

## Setup

1.  **Install Dependencies**:
    ```bash
    npm install
    ```

## Development

1.  **Start Local Server**:
    ```bash
    npm start
    ```
    Opens `http://localhost:3000`. Hot reloading is enabled.

2.  **Build for Production**:
    ```bash
    npm run build
    ```
    Generates static files in `build/`.

3.  **Serve Production Build**:
    ```bash
    npm run serve
    ```

## Project Structure

*   `docs/`: Course content (Markdown).
*   `src/`: React components and pages.
*   `docusaurus.config.js`: Site configuration.
*   `sidebars.js`: Navigation structure.
