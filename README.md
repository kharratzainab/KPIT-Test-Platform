# KPIT Test Platform

## Overview

This project creates a comprehensive simulation environment for ECU (Engine Control Unit) testing and diagnostics. It comprises three main components that work together to provide a complete testing ecosystem:

- **Doip_DiagnosticTool**: A diagnostic tool for reading and analyzing UDS (Unified Diagnostic Services) requests, monitoring ECU status, and forcing specific behaviors for testing purposes.
- **RealEcu**: A mock ECU simulator that emulates real ECU behavior for testing scenarios.
- **TestPlatform**: A testing platform that validates and executes test cases against the simulated ECU.

## Logging

All code modules implement Python's `logging` package to generate comprehensive trace logs for debugging, analysis, and traceability. Each component generates its own log files in the respective directories.

## Project Structure

```
KPIT-Test-Platform/
├── Doip_DiagnosticTool/
│   ├── src/                    # Source code for diagnostic tool
│   └── tests/                  # Test code for diagnostic tool
├── RealEcu/
│   ├── src/                    # Source code for ECU simulator
│   └── tests/                  # Test code for ECU simulator
└── TestPlatform/
    ├── src/                    # Source code for test platform
    └── tests/                  # Test code for test platform
```

## Getting Started

1. Clone the repository
2. Install dependencies: `pip install -r requirements.txt`
3. Review the documentation in each component's directory
4. Check log files in respective directories for execution traces

## Contributing

Please ensure all code follows the logging standards and includes appropriate trace logging for all major operations.
