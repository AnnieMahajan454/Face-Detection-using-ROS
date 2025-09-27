# Contributing to Face Detection using ROS

Thank you for your interest in contributing to this project! We welcome contributions from the community and are pleased to have you join us.

## How to Contribute

### Reporting Issues

1. **Search existing issues** to make sure your issue hasn't been reported
2. **Use the issue template** if available
3. **Provide detailed information** including:
   - Operating system and version
   - Python and OpenCV versions
   - Steps to reproduce the issue
   - Expected vs actual behavior
   - Screenshots or error messages if applicable

### Submitting Pull Requests

1. **Fork the repository** to your GitHub account
2. **Create a feature branch** from `main`:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Make your changes** following the coding standards
4. **Test your changes** thoroughly:
   ```bash
   python test_application.py
   ```
5. **Commit with descriptive messages** following our format:
   ```bash
   git commit -m "Add feature: Brief description
   
   - Detailed explanation of changes
   - Why the changes were necessary
   - Any breaking changes or migration notes"
   ```
6. **Push to your fork** and **create a pull request**

### Development Setup

#### Prerequisites
- Python 3.7 or higher
- OpenCV 4.0 or higher
- Git for version control
- (Optional) ROS Melodic/Noetic for full functionality

#### Setup Steps
1. **Clone your fork**:
   ```bash
   git clone https://github.com/YOUR_USERNAME/Face-Detection-using-ROS.git
   cd Face-Detection-using-ROS
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Run tests**:
   ```bash
   python test_application.py
   ```

4. **Test with camera**:
   ```bash
   python test_application.py
   # Select 'y' when prompted to run the demo
   ```

### Coding Standards

#### Python Code Style
- Follow **PEP 8** style guidelines
- Use **descriptive variable names**
- Add **docstrings** to all functions and classes
- Include **type hints** where appropriate
- Maximum line length: **88 characters** (Black formatter compatible)

#### Code Organization
- Keep functions **focused and small**
- Use **meaningful commit messages**
- Add **comments for complex logic**
- Follow the **existing project structure**

#### Example Code Style
```python
def detect_faces(self, gray_image: np.ndarray) -> np.ndarray:
    """
    Detect faces in the grayscale image.
    
    Args:
        gray_image: Grayscale image as numpy array
        
    Returns:
        faces: Array of face rectangles (x, y, w, h)
    """
    faces = self.face_cascade.detectMultiScale(
        gray_image,
        scaleFactor=self.scale_factor,
        minNeighbors=self.min_neighbors,
        minSize=tuple(self.min_size),
        maxSize=tuple(self.max_size)
    )
    return faces
```

### Testing Guidelines

#### Required Tests
- **All new features** must include tests
- **Bug fixes** should include regression tests
- **Cross-platform compatibility** testing when possible
- **Performance testing** for detection algorithms

#### Test Categories
1. **Unit Tests**: Individual function testing
2. **Integration Tests**: Node communication testing  
3. **System Tests**: End-to-end functionality
4. **Performance Tests**: Speed and accuracy metrics

### Documentation Standards

#### Code Documentation
- **Docstrings** for all public functions and classes
- **Inline comments** for complex logic
- **Type annotations** for function parameters
- **Examples** in docstrings when helpful

#### Project Documentation
- Update **README.md** for new features
- Update **CHANGELOG.md** for all changes
- Add **configuration examples** for new parameters
- Include **troubleshooting** for common issues

### Contribution Areas

We welcome contributions in these areas:

#### Algorithm Improvements
- **New detection algorithms** (MTCNN, YOLO, etc.)
- **Performance optimizations**
- **Accuracy improvements**
- **GPU acceleration** support

#### Platform Support
- **Additional OS support**
- **Docker containerization**
- **ROS2 compatibility**
- **Raspberry Pi optimization**

#### Features
- **Face recognition** capabilities
- **Multiple camera** support
- **Web interface** for remote access
- **Database integration**
- **Real-time tracking**

#### Documentation and Testing
- **Tutorial creation**
- **Video demonstrations**
- **Test coverage** improvements
- **Performance benchmarking**

### Community Guidelines

#### Be Respectful
- Use **welcoming and inclusive** language
- Respect **different viewpoints** and experiences
- Accept **constructive criticism** gracefully
- Focus on **what's best for the community**

#### Communication
- **Ask questions** if you're unsure about anything
- **Provide context** when reporting issues
- **Be patient** with response times
- **Help others** when you can

### Getting Help

#### Resources
- **GitHub Issues**: For bugs and feature requests
- **Discussions**: For questions and general discussion
- **Documentation**: README.md and WINDOWS_SETUP.md
- **Code Examples**: Check the test_application.py

#### Contact
- **GitHub Issues**: Primary support channel
- **Email**: For private or security-related matters
- **Documentation**: Check existing docs first

### Recognition

Contributors will be recognized in:
- **README.md** acknowledgments section
- **Release notes** for significant contributions
- **GitHub contributors** page
- **CHANGELOG.md** credit where applicable

Thank you for contributing to Face Detection using ROS! 🎉
