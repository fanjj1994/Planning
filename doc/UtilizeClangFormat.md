# How to Utilize .clang-format?

`clang-format` is a tool used to beautify and standardize code formatting. In this project, there is already a `.clang-format` file in the root directory, which contains the author's recommended formatting configurations.

- **Install Clang-Format**
    If you haven't installed `clang-format` yet, you can install it via a package manager. For example, on Ubuntu:
    ```bash
    sudo apt-get install clang-format
    ```

- **Apply Formatting**
    - To format a single file:
        ```bash
        clang-format -i /path/to/your/file.cpp
        ```
    - Use the `-style=file` parameter to ensure it reads the `.clang-format` file in the current or parent directory:
        ```bash
        clang-format -i -style=file /path/to/your/file.cpp
        ```
    - If you need to format multiple files, feel free to use `scripts/format_all.sh`.

- **Integrate with Editors**
    - **VSCode**: Install the [Clang-Format](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format) extension and ensure the settings point to the correct `.clang-format` file path.
    - **Other Editors**: Most modern IDEs and editors (such as JetBrains series, Vim, Emacs) have corresponding plugins or configuration options to support `clang-format`.

- **CI/CD Integration**
    Run `clang-format` checks in the continuous integration environment to ensure all committed code adheres to the formatting standards. You can add a command like the following in your CI script:
    ```bash
    git diff --name-only | xargs clang-format -i -style=file
    ```

- **Additional Tips**
    - **Pre-commit Hook**: You can set up a pre-commit hook to automatically format your code before each commit. Create a file named `pre-commit` in your `.git/hooks` directory with the following content:
        ```bash
        #!/bin/sh
        git diff --cached --name-only --diff-filter=ACM | grep '\.cpp\|\.h' | xargs clang-format -i -style=file
        git add .
        ```
        Make sure to give it execute permissions:
        ```bash
        chmod +x .git/hooks/pre-commit
        ```
    - **Batch Formatting**: To format all files in a directory, you can use:
        ```bash
        find /path/to/your/directory -name '*.cpp' -o -name '*.h' | xargs clang-format -i -style=file
        ```

By following these methods, you can effectively utilize the `.clang-format` file to maintain consistent code style.