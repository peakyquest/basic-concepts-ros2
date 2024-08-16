# Introduction to Python 3

Welcome to the Introduction to Python 3! This guide will help you get started with Python, a powerful and versatile programming language.

## Table of Contents
1. Introduction
2. Installation
3. Basic Syntax
4. Data Types
5. Control Structures
6. Functions
7. Modules and Packages
8. File Handling
9. Error Handling
10. Conclusion

## Introduction
Python is a high-level, interpreted programming language known for its readability and simplicity. It is widely used in web development, data analysis, artificial intelligence, scientific computing, and more.

## Installation
To install Python 3, follow these steps:
1. Download the installer from the official Python website.
2. Run the installer and follow the on-screen instructions.
3. Verify the installation by opening a terminal or command prompt and typing:
   ```sh
   python --version
   ```
## Basic Syntax
Python uses indentation to define code blocks. Here is a simple example:
   ```sh
   print("Hello, World!")
   ```
## Data Types
Python supports various data types, including:

Numbers: int, float, complex
1. Strings: str
2. Lists: list
3. Tuples: tuple
4. Dictionaries: dict
5. Sets: set

```
number = 10
text = "Hello"
my_list = [1, 2, 3]
```


### Control Structures
Python provides several control structures for managing the flow of the program:

1. Conditional Statements: if, elif, else
2. Loops: for, while

```
if number > 5:
    print("Number is greater than 5")
else:
    print("Number is 5 or less")
```

### Functions
Functions are defined using the def keyword:

```
def greet(name):
    return f"Hello, {name}!"
    
print(greet("Alice"))
```

### Modules and Packages
Modules are files containing Python code, and packages are directories containing multiple modules. You can import them using the import statement:


```
import math
print(math.sqrt(16))
```

### File Handling
Python allows you to read from and write to files:

```
with open("example.txt", "r") as file:
    content = file.read()
    print(content)
```


### Error Handling
Handle errors using try-except blocks:

```
try:
    result = 10 / 0
except ZeroDivisionError:
    print("Cannot divide by zero")
```
    

### Conclusion
This guide provides a brief overview of Python 3. For more detailed information, refer to the official Python documentation.

Happy coding!
