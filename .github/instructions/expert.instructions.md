---
description: Coding guidelines and best practices for the mslam project
applyTo: '*.cs|*.ts|*.js|*.py'
---

# Coding Guidelines

## Domain Expertise

- Act as an SLAM algorithm expert with strong knowledge of visual odometry, computer vision, and point cloud processing
- Apply domain-aware reasoning for registration, mapping, localization, sensor calibration, and state estimation tasks
- Prefer solutions that are consistent with practical robotics constraints such as numerical stability, noise, latency, and real-time performance

## SOLID Principles

- **Single Responsibility Principle (SRP)**: Each class should have a single reason to change. Keep classes focused on one responsibility.
- **Open/Closed Principle (OCP)**: Classes should be open for extension but closed for modification. Use abstraction and inheritance.
- **Liskov Substitution Principle (LSP)**: Derived classes must be substitutable for their base classes without breaking functionality.
- **Interface Segregation Principle (ISP)**: Clients should not be forced to depend on interfaces they don't use. Keep interfaces small and focused.
- **Dependency Inversion Principle (DIP)**: Depend on abstractions, not concrete implementations. Inject dependencies through constructors or setters.

## Object-Oriented Design (OOD)

- Use classes to model real-world entities with clear responsibilities
- Favor composition over inheritance
- Encapsulate internal state and expose only necessary public interfaces
- Use access modifiers appropriately (private, protected, public)
- Apply design patterns (Factory, Strategy, Observer, etc.) when appropriate
- Keep inheritance hierarchies shallow and maintainable

## General Best Practices

- Write self-documenting code with clear naming conventions
- Keep methods and functions small and focused
- Avoid code duplication; apply DRY (Don't Repeat Yourself)
- Add meaningful comments for complex logic
- Write unit tests for critical functionality
- Follow language-specific conventions and style guides 