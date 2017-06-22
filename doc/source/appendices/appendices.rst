.. _appendices:

.. only:: html

  .. title:: Appendices


.. only:: html

    Appendices
    ====================

    Programming Best Practices
    **************************

    This section describes tools and organizational strategies that would have made my life easier when I started programming.

    Motivation
    ----------

        * Engineering software development is multiphysics/multidisciplinary.
        * Many of us develop expertise and simulation software to model/simulate a particular domain or length scale.
        * It is not possible for a single individual or group to develop code for all domains and length scales, therefore *code reusability* should emphasized

    Code Reusability
    ----------------

        * We use a variety of programming languages (For numerical computation C/C++, Python, Fortran, MATLAB are common).
        * We cannot and should not restrict all software development to use the same language.
        * Instead it is recommended to work in your language of choice, but to facilitate collaboration by:

              * Providing a basic command-line tool that allows users to set up their own models.
              * Providing a documented Application Programming Interface (API) to allow others to incorporate your functions in to their own code AT LEAST in the language in which the original functions were programmed.

    Coding Style
    ------------

        * Write code as if it is part of your dissertation, even if it will not be.
        * Use meaningful, descriptive variable names (in English!). Assume that others will be critically reviewing your code later. Most likely it will be you looking at your code a month from now trying to track down a bug.
        * Avoid having one gigantic 10,000 line ```main()``` function.
        * Clearly organize collections of functions in to logical groupings - ideally each file should be less than 2000 lines.
        * Your future employer may judge you on how well-organized your code is.
        * You may not be collaborating with others now, but you most likely will be in your future career. Get in the habit of following a style guide for your programming language of choice and use consistent formatting throughout your code.

        .. figure:: code_quality.png
            :width: 600px
            :align: center

            credit: xkcd

    Documentation
    -------------

        * It is not enough to simply create the code or provide the API. No matter how amazing your code is, no one will use it if they cannot find good examples or tutorials of how to use your code. Fortunately there are free, open-source tools to help you generate documentation.
        * `Doxygen <http://www.stack.nl/~dimitri/doxygen/>`_ and `Sphinx <http://sphinx-doc.org/>`_ allow you to automatically generate clearly formatted HTML and PDF documentation by writing comments inline with source code. Even if you do not plan to share your code in the future, the documentation will help you remember what a function is supposed to do a month from now when you are trying to track down a bug.
        * Other resources:

            * `LaTeX <http://www.latex-project.org/>`_
            * `Markdown <http://www.latex-project.org/>`_
            * `Restructured Text <http://docutils.sourceforge.net/docs/ref/rst/restructuredtext.html>`_

    Version Control
    ---------------

        * *Now* is a good time to learn `git <http://www.git-scm.com>`_
        * I suggest to use git from the very beginning of code development.
        * Using git via the command line has a steep learning curve, but many benefits. There are also graphical user interfaces or plugins for your text editor to help you issue git commands, but I recommend starting out with the command line, so you understand what the graphical user interfaces are doing under the hood.

        * Advantages of working with git for an individual:

            * Allows you to take regular snapshots of your code. If you break your code, you can take any file or the entire project back in time to any previous *commit*.
            * The entire project history is stored in a distributed fashion. Every computer that has cloned the repository has a complete history of the project. If the server goes down, you can recreate the repository from any computer.

        * Advantages to working with git for an organization:

            * Git allows you to maintain multiple branches of code (for example a stable *master* and unstable *development* branch.)
            * Allows multiple people to work on the same code base.

    Programming Getting Started Guide
    *********************************

    I am often asked how to get started in programming. One difficult task is to decide what programming tools and editing environment to select from the myriad of options. I am a proponent of using, as much as reasonably possible, free and open source software. Therefore, the tools I recommend fall *mostly* under this category.  This is far to be a comprehensive list, rather it provides a brief overview of a few general purpose tools that I have had direct experience and success with.

    Setting up an Integrated Development Environment (IDE)
    ------------------------------------------------------

    An advantage to the editors listed below is that they can be used to code in virtually any programming language. Plugins are available to get code highlighting and automatic code completion for most languages.

        * Tools with a Graphical User Interface (GUI)

            - My personal favorite and recommendation to new programmers is the `Atom <http://atom.io/>`_ text editor. Atom is developed by the people at `Github <http://github.com/>`_ . It is free and open source. There are many plugins available to extend its functionality. Atom is available for Windows, Mac and Linux.
            - `Sublime Text 3 <http://www.sublimetext.com/3>`_ is another popular cross-platform editor that is beautiful and (through plugins) highly extensible. Sublime has an unlimited evaluation period so you can try it out before you buy it (the program will remind you periodically that you should buy it). Although Sublime Text is nonfree and closed-source, Sublime Text is a labor of love from a single developer, not a large corporation, and it is nice to support projects like this. One of the advantages to Sublime over Atom is that it is programmed in C++/Python (while Atom is mostly programmed in Javascript). Therefore, Sublime has better performance for  operations like opening large files. The Sublime editor has also been around longer than Atom, so there are currently more plugins available for Sublime, although the situation is changing rapidly: many new plugins for Atom are added every week.
            - `Microsoft Visual Studio <https://www.visualstudio.com/>`_ Microsoft's new editor, has a layout and many features inspired by Sublime Text. Microsoft's editor also works on Linux and Mac. The new editor is part of Microsoft's new initiative to embrace Free and Open Source software, something Microsoft does not have a good track record with.
            - `Eclipse <http://www.eclipse.org/>`_ Eclipse is a free and open source editor primarily targeted to those developing code in Java -- the editor itself is written in Java, although plugins are available to program in other languages. I personally find the layout and user interface of Sublime and Atom more appealing than Eclipse.

        * Command Line Development Environments (for advanced users)
            - `VIM <http://www.vim.org/>`_
            - `emacs <https://www.gnu.org/software/emacs/>`_

    I suggest you try a few different editors when you first start programming. Once you find one you like, stick with it and try to use the same tool for all your programming projects. The following discussion focuses on extending the functionality of Atom and Sublime Text by installing plugins. Plugins can be installed in other editors, refer to the manual pages for instructions. Below are some Atom and Sublime Text plugins I find useful for editing Python, C/C++, Javascript, HTML, JSON, CSS, LaTeX, Restructured Text and Markdown files:

        * Atom Plugins
            Cmd-, brings up the preferences menu. Click on the install button, then search for a package and click install. Packages I commonly use include:

            - autocomplete-bibtex
            - autocomplete-clang
            - language-cmake
            - language-rst
            - latex
            - linter
            - linter-chktex
            - linter-clang
            - linter-flake8
            - minimap
            - pdf-view
            - project-manager

        * Sublime Text Plugins
            Install the Sublime Text package manager. Follow these `instructions <https://packagecontrol.io/installation>`_.
            Then to install a package, type Shift-Ctrl-P, then type ''install'', select ''Package Control: Install Package''.
            Type the name of the package to install, select it and press enter.

            - Sublime Linter
            - JSON Linter
            - JSON reindent
            - Python Linter - flake8
            - Sidebarenhancements

    Resources
    ---------

    `Stack Overflow <http://stackoverflow.com/>`_ is an excellent place to ask questions.  Most common questions have already been answered, so take the time to search if your question has already been answered before posting a new one. Also please read `How to Ask Questions the Smart Way <http://catb.org/~esr/faqs/smart-questions.html>`_ before posting questions online.

    Introduction to Python
    **********************

    Python is an elegant, general-purpose scripted programming language. For engineers and scientists interested in programming, I strongly recommend Python as a first language to learn. Although not as fast as C/C++, its simple syntax makes it suitable for rapidly prototyping algorithms for scientific/engineering applications. Python is also suitable for applications where speed and memory management are not as critical, such as for user interfaces. Depending on the application, it may be possible to do all development in Python. If speed is paramount, many of the programming constructs in Python will translate to compiled languages such as C and C++.  Python is *Dynamically Typed* and comes with a variety of powerful and convenient built-in datastructures (discussed below), other aspects that make Python well-suited for quickly developing and deploying applications.

    You most likely already have `Python <https://python.org>`_ installed on your machine. To find out, go to the command line. Open your terminal application in Linux/Mac or in Windows go to Start -> run -> cmd, then type 'python'. You may also have python3 installed on your machine. If you do not have python or python3, both Python 2 and 3 can be installed through the `Python <https://python.org/>`_ website. The following section contains a brief introduction to Python programming constructs.

    Why Python?
    -----------

        * Scripted language with simple syntax
        * Based on C/C++ programming language
        * *Dynamically* typed
        * A vibrant community
        * Python design principles - the zen of python (type the below into an interactive Python session)

    .. code:: python

        import this


    Basics of Python Program
    ------------------------

        * Everything in Python is an object.
        * No trailing semicolons like in C, Javascript, etc.
        * Indentation is important!

    An example Python program:

    .. code:: python

        #!/usr/bin/env/python

        import numpy as np
        from time import localtime

        def greet(name):
            print("hello " + name)

        if __name__ == "__main__":
            greet("world")


    Data Structures
    ---------------

    Python is *dynamically* typed. There is no need to declare the type of a variable. Python figures it out for you:

    .. code:: python

        a = 1        #  an integer
        b = 2.0      #  a double precision (real number)
        c = a / b
        d = "hello"  #  a string
        e = True     #  a boolean


    `Lists <https://docs.python.org/2/tutorial/datastructures.html>`_

    .. code:: python

        mylist = [1, 2, 3, 5, 4]
        mylist.append(2)
        print(mylist[0])
        print(mylist[-1])
        print(mylist[:2])
        mylist.sort()
        print(mylist)


    `Dictionaries <https://docs.python.org/2/tutorial/datastructures.html#dictionaries>`_ Below example borrowed from Python website.

    .. code:: python

        tel = {'jack': 4098, 'sape': 4139}
        tel['guido'] = 4127
        print(tel)
        print(tel['jack'])
        del tel['sape']
        print(tel.keys())
        print('guido' in tel)


    The dict() constructor builds dictionaries directly from sequences of key-value pairs:

    .. code:: python

        dict([('sape', 4139), ('guido', 4127), ('jack', 4098)])


    Programming Constructs
    ----------------------

    If statements:

    .. code:: python

        print("1. do something")
        print("2. do something else")
        print("3. quit")

        user_input = input("Please make a valid selection: ")
        if user_input == "1":
            print("doing something...")
        elif user_input == "2":
            print("doing something else")
        elif user_input == "3":
            quit()
        else:
            print("not a valid selection")


    For loops:

    .. code:: python

        def ho(n=3):
            for i in range(n):
                print("ho ", end = "")
        ho()
        ho(n=4)


    While loops:

    .. code:: python

        while True:
            n = input("Please enter 'hello':")
            if n.strip() == 'hello':
                break


    Functions returning a single value:

    .. code:: python

        def myfunction(a, b, optional_argument="default_value"):
            print("you entered the following string: "
                  + optional_argument)
            return a + b


    Functions returning multiple values in a *tuple*:

    .. code:: python

        def myfunction2(a, b):
            return a, b


    Exercise: Write a function that takes in 4 numbers, adds the first two numbers and multiplies the second two numbers, and returns the results in a tuple.

    File IO
    -------

    Exercise: Write a `JSON <http://json.org/>`_ file using your text editor. Include as many types of data as you would like.

    Load the text file in to a Python dictionary:

    .. code:: python

        import json

        with open("myfilename.json", "r") as f:
            mydata = json.load(f)

        # print the keys()
        print(mydata.keys())

        mydata["somekey"] = 3

        # access a value at a specific key
        print(mydata["somekey"])

        with open("outfile.json", "w") as f:
            json.dump(mydata, f)


    Classes
    -------

    *Classes* are used to describe your own object. `Look here <https://docs.python.org/2/tutorial/classes.html>`_ for more information.

    .. code:: python

        class Complex:
            def __init__(self, realpart, imagpart):
                self.r = realpart
                self.i = imagpart

            def print(self):
                print("real part: " + self.r)
                print("imaginary part " + self.i)

        # create an instance of your new object
        x = Complex(3.0, -4.5)

        # use the 'print' method in your object
        x.print()


    Useful Python Libraries
    -----------------------

    Once you have familiarized yourself with basic Python, you can install Python packages to extend the basic functionality using the pip package manager.

        * `pip <http://docs.python-guide.org/en/latest/starting/install/win/>`_
            A package manager for Python, used to install external python libraries from the internet repositories. Matplotlib and Numpy (below) are examples of libraries that can be installed using pip

        * `Matplotlib <http://matplotlib.org>`_
            A Python plotting library.

        * `Numpy/Scipy <http://www.numpy.org/>`_
            A numerical library for Python. Most functions are wrappers for C or Fortran code, so they are computationally efficient.

        * `Graphviz <http://graphviz.org/>`_
            A graph/network visualization tool. A python interface to graphviz is available `here <https://pypi.python.org/pypi/graphviz>`_

    Quickstart Guide to Setting up Sphinx
    *************************************

    This website was generated using Sphinx, a tool for creating documentation with the following capabilities:

        * Many output formats, including HTML (including Windows HTML Help), LaTeX (for printable PDF versions), ePub, Texinfo, manual pages, plain text
        * Extensive cross-references: semantic markup and automatic links for functions, classes, citations, glossary terms and similar pieces of information
        * Hierarchical structure: easy definition of a document tree, with automatic links to siblings, parents and children
        * Automatic indices: general index as well as a language-specific module indices
        * Code handling: automatic highlighting using the Pygments highlighter
        * Extensions: automatic testing of code snippets, inclusion of docstrings from Python modules (API docs), and more
        * Contributed extensions: more than 50 extensions contributed by users in a second repository; most of them installable from PyPI

    Source files are written in `Restructured Text (RST) <http://docutils.sourceforge.net/docs/ref/rst/restructuredtext.html>`_ . Compared to LaTeX, the syntax is simpler. RST files are designed to be human readable, similar to a plain text email. Click the *View page source* button at the top of any page on this site to see the accompanying source code.

    One of the advantages of Sphinx is that content is kept separate from the formatting, allowing multiple formats to be generated from the same source. For example, a website, an ebook, and a PDF can all be created from the same RST files.


    Installation
    ------------

    First install sphinx on your system.

    On arch linux::

        pacman -S python-sphinx

    On ubuntu (debian-based) systems::

        apt-get install python-sphinx

    For Mac and Windows, follow the instructions `here <http://sphinx-doc.org/latest/install.html>`_:
    To get figure numbering and the ability to cite references, you need the following two packages:

    `numfig <https://pypi.python.org/pypi/sphinx_numfig/0.1.1>`_ ::

        pip install sphinx_numfig


    `sphinxcontrib-bibtex <http://sphinxcontrib-bibtex.readthedocs.org/en/latest/>`_ ::

        pip install sphinxcontrib-bibtex


    Creating a new Sphinx Project
    -----------------------------

    Create a directory for your sphinx project by the issuing the following commands from your terminal (linux commands, modify for windows)::

        mkdir sphinx
        cd sphinx


    Sphinx comes with a quickstart script to initialize a new Sphinx project::

        sphinx-quickstart


    Answer the interactive dialog to set up your new Sphinx project. Once you have set up your project, you can build the html documentation::

        make html

    Type 'make' to see the full list of output options.

    Create a new file in the directory called "introduction.rst"::

        .. title:: Introduction

            Introduction
            ============

                Some text


    Modify your index.rst file in your text editor::

        Full Title
        ==========

        .. toctree::
            :maxdepth: 2

            introduction

    Build your project again::

        make html


    You can find the html output under build/html/index.html

    I will add more instructions on how to add citations/figures later.



    Accessing My Git Repository
    ***************************

    My ongoing PhD research (source code, publications, references and documentation) is stored in a private git repository which is accessible by ssh at the web address you are currently viewing this site from.

    To access, you must have an ssh username and password set up.

    This guide assumes you are connecting from a unix machine.

    Set up your ~/.ssh/config
    -------------------------

    Edit your ~/.ssh/config Replace emacs with your editor of choice (i.e. *subl*, *nano*, *vim*, etc.)

    .. code:: bash

        emacs ~/.ssh/config


    Add the following entry, replacing username with the username provided to you for access to the system (this is not the username on your local machine).


    .. code:: bash

        Host kristopher

            HostName viper.ahmct.ucdavis.edu
            User <username>


    Save and exit your editor.

    Generate an ssh key
    -------------------

    Then generate an ssh key:

    .. code:: bash

        ssh-keygen


    You can follow the prompts and opt not to use a password. For extra security, you may choose to use an ssh key together with a password, however this requires that you enter that password each time you perform a git push/pull.

    Once you have generated your ssh key, you can copy the *public* half of your key to my server by issuing the ssh-copy-id command. On most linux distributions, this command is available. On a mac, if you don't already have the ssh-copy-id command, you can install it by following the instructions `here <https://github.com/beautifulcode/ssh-copy-id-for-OSX>`_

    .. code:: bash

        ssh-copy-id kristopher


    Note the *kristopher* shortcut is the alias you set up in your ssh config above. With this shortcut, you do not need to remember the full address to my ssh server. If for some reason in the future the address changes, you can just update the alias in your ~/.ssh/config file.

    When you execute the ssh-copy-id command, you will be asked for the password provided to you to access the server. Once you have entered the password and your public key is successfully installed, you will not need to enter your password to login by ssh. You can test your configuration by doing:

    .. code:: bash

        ssh kristopher


    You should connect to the remote machine. Exit back out of the machine by entering:

    .. code:: bash

        exit


    Clone the git repository
    ------------------------

    Now clone the git repository. Change directory on your machine to where you want the repository to reside. For example I keep all of my git repositories under my *Projects* folder in my home directory.

    .. code:: bash

        cd ~/Projects
        git clone kristopher:~/../git/phd.git


    You now have the latest version of the git repository. To retrieve a new update, perform a git pull from within the phd directory.

    .. code:: bash

        git pull


    Building the STAR Program
    *************************

    Dependencies
    ------------

    STAR uses a few free and open source libraries that are not included with the source code. The following dependencies must first be installed on your system to build the STAR executables:

      * `Assimp <http://assimp.sourceforge.net/>`_
      * `Jansson <http://www.digip.org/jansson/>`_
      * `Qt5 <http://www.qt.io/developers/>`_
      * `Graphviz <http://www.graphviz.org/>`_
      * `Cblas <http://www.netlib.org/blas/>`_

    Macintosh
    ---------
    On a Macintosh, the dependencies can be installed using the `brew <http://brew.sh/>`_ package manager.

    .. code:: bash

        brew install assimp jansson qt5 graphviz

    The cblas package is already available on a Macintosh using the *accelerate* framework, the CMakeLists.txt file may need to be modified to bring in this dependency correctly.

    Arch Linux
    ----------

    On Arch Linux, the dependencies can be installed using the pacman package manager:

    .. code:: bash

        pacman -S assimp jansson qt5 graphviz cblas


    Ubuntu and other Debian-based Linux Distributions
    -------------------------------------------------

    .. code:: bash

        apt-get install assimp jansson qt5 graphviz cblas


    Windows
    -------

    While theoretically the Qt5 framework should allow building the application on
    Windows, I don't use Windows and therefore have not tested building or running the
    executables in a Windows environment. The dependencies may need to be
    installed manually and the CMakeLists.txt file may need to be modified to set
    the include directories and compile flags correctly.

    Installation
    ------------

    From within the top directory of the repository, issue the following command:

    .. code:: bash

        mkdir build
        cd build
        cmake ..
        make


    Executables will be located in
    ${PROJECT_DIRECTORY}/build/bin

    Where ${PROJECT_DIRECTORY} is the root directory of the git project on your
    system.

    You can test if the executable works correctly by running the command line interface to the program as:

    .. code:: bash

        cd ${PROJECT_DIRECTORY}/build/bin
        ./star_cli ../models/excavator/excavator.star


    Run the graphical user interface component of the program by executing the following command from within the ${PROJECT_DIRECTORY}/build/bin directory:

    .. code:: bash

        ./star


    Documentation
    -------------

    Documentation can be built by issuing the command:

    .. code::

        make doc


    Optional: documentation can be copied to your /share/doc folder by issuing the command

    .. code::

        make install


    Alternatively, refer `here <http://viper.ahmct.ucdavis.edu/static/doxygen/>`_ for the latest Doxygen generated documentation.

    Notes on OpenGL / Qt Compatibility
    **********************************

    About OpenGL Function Loading
    -----------------------------

        * OpenGL Functions are loaded dynamically at run time. The OpenGL functions that you call are actually function pointers that point to functions specific to your GPU device.

        * You need an extensions manager to set the function pointers. Popular extensions managers include.

            - GLEW (GL Extensions Wrangler)

            - GL3W, recommended by the OpenGL Superbible (aka the blue book).

            - GLUT (GL Utility Toolkit)


    OpenGL on Qt
    ------------

        * Qt includes its own extensions manager. By default it only includes functions for OpenGL ES 2, a subset of OpenGL that is compatible with both desktop and mobile devices.

        * GLEW, GL3W, GLUT do not work with Qt! All OpenGL functions are loaded at the top level in both extensions managers, which means you get namespace collisions if you try to use two extensions managers together in the same project.

        * To get OpenGL 3.3+ Core functionality in Qt, you need to specifically request it.

        * You also need to check if your graphics card/driver support the version and *gracefully* handle exceptions. My laptop GPU (Intel integrated graphics with open source Mesa driver) supports up to OpenGL 3.3. My desktop GPU (AMD Radeon 6800 with proprietary closed-source Catalyst driver) supports up to 4.3. It is recommended to pick a minimum baseline version of OpenGL that your application requires.


    Qt Example code
    ---------------

    OpenGL functions must be initialized after an OpenGL context is created.

    .. code-block:: c

        context = new QOpenGLContext(this);
        context->setFormat(requestedFormat());
        context->create();
        m_funcs = context->versionFunctions<QOpenGLFunctions_3_3_Core>();
        m_funcs->initializeOpenGLFunctions();


    Why do you need OpenGL 3.3+?
    ----------------------------

        * 3.3+: Uniform Buffer Blocks were introduced in 3.3. Uniform Buffer Blocks are used for data that remain constant for every shader program.  You can set quantities in Uniform Buffer Blocks once and switch in and out shader programs without having to reload uniform data. You also have granular control, you can set data at a single index within a buffer block without having to reload all the data. I plan to use Uniform Buffer Blocks for lights.

        * 4.0+: Allows you to switch out Shader (GLSL) subroutines, which is faster than switching an entire program. The graphics pipeline typically consists of the following shaders:

            - Vertex Shader

            - Geometry Shader (optional)

            - Fragment Shader

        * 4.3+: Compute Shaders allow you to do GPGPU programming on the same data used to display graphics and offers better synchronization than CUDA or OpenCL.  In the future, will be useful for contact detection and camera effects like depth-of-field or motion blur.


    Notes on Dynamic Memory Allocation
    **********************************

    Certain aspects of a program are static (such as the function definitions that comprise the program). On the other hand, if a program dynamically loads or generates data at run time (i.e. most programs), the data must be stored in dynamic memory. For example, when a mechanism is loaded in the STAR 2.0 software, its topology and geometry are loaded from external text files. There is no way for the program to know ahead of time how large the data will be, and therefore space for data must be allocated dynamically.

    There are two mechanisms for storing dynamic data, the stack and the heap.

    The Stack
    ---------

    The *stack* is a fixed size block of fast memory that is located physically closer to the CPU's floating point processor cores. Note that on both the CPU and GPU, the random access memory is arranged in hierarchies with the smallest and fastest memory located physically closer to the floating point processing cores. Memory for the stack is allocated for each *thread*, a program's basic unit of operation. A multithreaded, parallel program would have multiple threads, with a unique stack allocated for each thread. As the stack is a first-in-first-out (FIFO) data structure that is accessed by a single thread, limited synchronization is required to store data there. Data on the stack is local only to the calling function, and will be destroyed once the function completes its execution.

    Example of allocating data on the stack:

    .. code-block:: c

        void foo() {
          double stackData[3]; // allocates data on the stack
          for(int i = 0; i < 3; i++) {
            stackData[i] = i;
          }
          for(int i = 0; i < 3; i++) {
            printf("%f ", stackData[i]);
          }
          printf("\n");
        }

    Note that the variable *stackData* is not accessible to other functions. Once the function foo() terminates, the variable is destroyed. The advantages and disadvantages to using the stack are summarized below.

    Advantages:

        * Fast data storage suitable as a workspace for temporary operations

    Disadvantages:

        * Limited size
        * Memory is freed/deleted when function completes execution
        * Cannot be used/accessed by other threads or from functions at higher levels within a program


    The Heap
    --------

    The *heap* is the primary storage location for dynamically allocated memory. Memory allocated on the heap will be available for other functions to access, even at functions at higher levels within a program. The heap is accessible from multiple threads, and therefore requires greater synchronization to avoid overwriting data. Unlike the stack, the heap is not FIFO, and requires more complicated logic for finding free memory locations for new memory allocations. In practice, memory access to the heap is quite fast, but for the reasons stated above account, memory reading and writing to the heap is *relatively* slower than the stack. In the C programming language, memory in the heap must be allocated using calls to *malloc* or *calloc* and must be freed by calling *free*. Below is an example of dynamically allocating memory on the heap.

    .. code-block:: c

        double* bar() {
          double* heapData = malloc(3 * sizeof(double));
          for(int i = 0; i < 3; i++) {
            heapData[i] = i;
          }
          for(int i = 0; i < 3; i++) {
            printf("%f ", heapData[i]);
          }
          printf("\n");
          return heapData;
        }


    bar() achieves the same functionality as foo(), but the variable data *is* available to other functions. In the C programming language, memory that is dynamically allocated must be freed by the user. Once the user is finished with the heapData variable, it should be freed as:

    .. code-block:: c

        free(heapData);


    Memory Leaks
    ------------

    If the address to memory allocated on the heap becomes no longer available to the program because it is not returned from a function or is overwritten, the data cannot be freed. This situation is known as a *memory leak*. You don't need to worry about memory leaks for data on the stack -- memory on the stack will be freed automatically when the function terminates. You also need not worry about freeing memory for static quantities, such as function definitions, which are loaded at the beginning of program execution and released when the program terminates.


    Garbage Collection
    ------------------

    *Garbage collection* is a mechanism to automatically free memory on the heap when the program detects that the pointer to the memory location is no longer accessible from the main program (i.e. *goes out of scope*), obviating the need for the programmer to call free() to release the memory from the program. Garbage collection is designed to make the job of programming easier, but does not improve performance of a program. Indeed, garbage collection adds complexity and overhead to a program, as in addition to performing the instructions set in the program, the program must also monitor itself to determine when quantities are no longer necessary.

    Languages such as Python, Javascript, Java, etc. perform varying degrees of garbage collection, adding overhead to the program. Note that the C Programming language contains no concept of garbage collection, and requires the user to explicitly free memory that has been allocated on the heap. As C and C++ provide more tools to explicitly manage memory than other languages, they are most commonly used for performance critical applications.


    LU Decomposition Numerical Example
    **********************************

    Gauss Elimination & LU Decomposition
    ------------------------------------

    When solving a system of linear equations by hand, it is common to use Gauss or Gauss–Jordan elimination, which should be familiar from undergraduate linear algebra. With the advent of computers in the 1940s, automated techniques for solving a set of linear equations were developed. In 1948, Alan Turing extended Gauss elimination to perform LU decomposition :cite:`turing1948`. With a minor variation to Gauss elimination, you can get the lower triangular L matrix factor as a byproduct of Gauss elimination.

    Row–pivoting is often performed for numerical stability purposes :cite:`turing1948`. In addition to avoiding round–off errors, row pivoting also avoids zero pivots. Note that for
    very large matrices, roundoff error is inevitable and an alternative (and more expensive) algorithm to factor a matrix may be required, depending on the application.

    Partial Pivoting Example
    ------------------------

    When factoring square matrices, LU decomposition is generally performed with partial pivoting (i.e. only row or only column interchanges — or permutations). If partial pivoting is used, the original matrix :math:`A` in the system of equations :math:`Ax=b` is not factored to :math:`LU` directly. Instead, the partial pivoting factorization yields

    .. math::
        :label: eq:PALU

        P_RA=LU


    where :math:`P_R` is an orthonormal permutation matrix that when left–multiplied with A reorders the rows of :math:`A`. Mahaffy performs LU decomposition in his example without partial pivoting. Read and understand his example first, then continue to see how his example can be modified to include full row and column pivoting to solve the same system of
    equations for :math:`\underline{x}` below.

    .. math::
        :label: eq:step1

        \begin{bmatrix}
        1 & 2 & -1\\
        4 & 3 & 1\\
        2 & 2 & 3
        \end{bmatrix}
        \begin{Bmatrix}
        x_1 \\ x_2 \\ x_3
        \end{Bmatrix}=
        \begin{Bmatrix}
        2 \\ 3 \\ 5
        \end{Bmatrix}


    Then, form the augmented matrix as:

    .. math::
        :label: eq:augmented

        \left[
        \begin{array}{ccc|c}
        1 & 2 & -1 & 2 \\
        4 & 3 & 1 & 3 \\
        2 & 2 & 3 & 5
        \end{array}
        \right]


    To determine the equation used to start Gauss elimination, divide the coefficients of column 1 by the largest value in each respective row of matrix :math:`A`. The row with the largest absolute value is chosen as the first pivot equation. Rows 1–3 have values of 1/2, 4/4 and 2/3 respectively. Therefore, row 2 is selected as the pivot equation and is
    permuted to the top position. This permutation is recorded so as to be able to construct the :math:`P` matrix from Eq. :eq:`eq:PALU` at the end of the elimination process.

    .. math::
        :label: eq:P_R1

        P_{R1}=\begin{bmatrix}
        0 & 1 & 0\\
        1 & 0 & 0\\
        0 & 0 & 1
        \end{bmatrix}


    The modified system of equations becomes

    .. math::
        :label: eq:modified

        \left[
        \begin{array}{ccc|c}
        4 & 3 & 1 & 3 \\
        1 & 2 & -1 & 2 \\
        2 & 2 & 3 & 5
        \end{array}
        \right]


    Next eliminate the entries in the first column of the second and third rows by subtracting multiples of the first row, yielding

    .. math::
        :label: eq:U1

        \left[
        \begin{array}{c:cc|c}
        4 & 3 & 1 & 3 \\
        \hdashline
        1-4/4 & 2-3/4 & -1-1/4 & 2-3/4 \\
        2-4/2 & 2-3/2 & 3-1/2 & 5-3/2
        \end{array}
        \right]=
        \left[
        \begin{array}{c:cc|c}
        4 & 3 & 1 & 3 \\
        \hdashline
        0 & 1.25 & -1.25 & 1.25 \\
        0 & 0.5 & 2.5 & 3.5
        \end{array}
        \right]
        \label{eq:U1}


    The section above and to the left of the dashed lines is done and does not change for the remainder of the factorization. Continue similarly, processing the remaining equations.

    Divide the entry in the second column of rows 2 and 3 by the maximum value in each respective row to determine the next candidate equation. Rows 2 and 3 have values of -1 and 1/5 respectively, therefore row 2 is the second pivot equation. Technically, the second permutation matrix
    :math:`P_{R2}` looks like

    .. math::
        :label: eq:P_R2

        P_{R2}=\begin{bmatrix}
        1 & 0 & 0\\
        0 & 1 & 0\\
        0 & 0 & 1
        \end{bmatrix}


    although in this case the result is not very interesting.

    Eliminate the term in the third row, second column by subtracting
    multiples of equation 2.

    .. math::
        :label: eq:U

        \left[
        \begin{array}{cc:c|c}
        4 & 3 & 1 & 3 \\
        0 & 1.25 & -1.25 & 1.25 \\
        \hdashline
        0 & 0.5-1.25/2.5 & 2.5-(-1.25/2.5) & 3.5-1.25/2.5
        \end{array}
        \right]=
        \left[
        \begin{array}{cc:c|c}
        4 & 3 & 1 & 3 \\
        0 & 1.25 & -1.25 & 1.25 \\
        \hdashline
        0 & 0 & 3 & 3
        \end{array}
        \right]


    At this point, we are finished with Gauss elimination and the system of
    equations can be solved by back substitution.

    .. math::
        :label: eq:back_sub1

        x_3=3/3=1


    .. math::
        :label: eq:back_sub2

        x_2 = (1.25 +1.25 x_3)/1.25, x_2=2


    .. math::
        :label: eq:back_sub3

        x_1 = (3 -3x_2-x_3)/4, x_1=-1


    Additionally, the final row permutation matrix :math:`P_R` is given as

    .. math::
        :label: eq:P_R_final

        P_R=P_{R2}P_{R1}=\begin{bmatrix}
        0 & 1 & 0\\
        1 & 0 & 0\\
        0 & 0 & 1
        \end{bmatrix}


    Note that only Gauss elimination was performed in Eq. :eq:`eq:U` to get the U matrix factor of A. The L matrix can be found by an additional book–keeping operation. As multiples of the pivot equation are subtracted from the remaining equations, the multiples that were subtracted are stored in the lower diagonal portion of the factored matrix. Therefore, after Eq. :eq:`eq:U1`, the composite matrix becomes

    .. math::
        :label: eq:composite1

        \left[
        \begin{array}{c:cc|c}
        4 & 3 & 1 & 3 \\
        \hdashline
        {\color{red}{0.25}} & 1.25 & -1.25 & 1.25 \\
        {\color{red}{0.5}} & 0.5 & 2.5 & 3.5
        \end{array}
        \right]


    and after Eq. :eq:`eq:U` becomes.

    .. math::
        :label: eq:composite

        \left[
        \begin{array}{ccc|c}
        4 & 3 & 1 & 3 \\
        {\color{red}{0.25}} & 1.25 & -1.25 & 1.25 \\
        {\color{red}{0.5}} & {\color{red}{0.4}} & 3 & 3
        \end{array}
        \right]


    The composite matrix :math:`L\cup U` is stored in matrix
    :eq:`eq:composite`.

    .. math::
        :label: eq:L

        L=\begin{bmatrix}
        1 & 0 & 0 \\
        .25 & 1 & 0 \\
        0.5 & 0.4 & 1
        \end{bmatrix}


    and

    .. math::
        :label: eq:U2

        U=\left[
        \begin{array}{ccc}
        4 & 3 & 1 \\
        0 & 1.25 & -1.25\\
        0 & 0 & 3
        \end{array}
        \right]


    As an excercise you can verify that multiplying :math:`LU` together yields :math:`P_RA`.

    Typically, one does not just solve a system of equations
    :math:`A\underline{x}=\underline{b}` once, but the system of equations might be solved for many times for different values of . LU factorization (and also the Gauss elimination algorithm) requires :math:`\mathcal{O}(n^3)` operations, whereas backsubstitution using the L and U matrices requires only :math:`\mathcal{O}(n^2)` operations.  Therefore, computing L and U up front and reusing them for subsequent calculations is faster than repeated Gauss elimination for a new value of :math:`\underline{b}` when the matrix :math:`A` is unchanged.


    Gauss–Jordan Elimination and LDU Decomposition
    ----------------------------------------------

    Jordan made a minor modification to Gauss elimination to make the terms in the upper diagonal matrix, U go to unity. In this section it is shown that LDU decomposition is analagous to LDU decomposition, with an extra book-keeping operation applied.  To make the diagonal terms go to unity in Eq. :eq:`eq:U2`, divide all terms in the respective row by the first non-zero entry in the that row. The first row will be divided through by 4, the second row will be divided by 1.25, and the third row will be divided by 3. At the end of this process, the U matrix looks like:

    .. math::
        :label: eq:U2

        U=\left[
        \begin{array}{ccc}
        1 & 0.75 & 0.25 \\
        0 & 1 & -1\\
        0 & 0 & 1
        \end{array}
        \right]



    The terms along the diagonal of the new :math:`U` matrix are always unity, so let us use that space to record the value that each equation was multiplied by in the factored/permuted matrix.

    .. math::
        :label: eq:LDU_composite

        L\cup D\cup U=\left[
        \begin{array}{ccc}
        {\color{blue}{0.25}} & 0.75 & 0.25 \\
        {\color{red}{0.25}} & {\color{blue}{0.8}} & -1\\
        {\color{red}{0.5}} & {\color{red}{0.4}} & {\color{blue}{0.33}}
        \end{array}
        \right]


    The composite matrix :math:`L \cup D \cup U` now contains the matrix
    factors :math:`L`, a lower triangular matrix with 1s along the diagonal,
    :math:`D` a diagonal matrix, and :math:`U` an upper triangular matrix
    with 1s along the diagonal:

    .. math::
        :label: eq:jordan_L

        L=\begin{bmatrix}
        1 & 0 & 0 \\
        .25 & 1 & 0 \\
        0.5 & 0.4 & 1
        \end{bmatrix}


    .. math::
        :label: eq:jordan_D

        D=\left[
        \begin{array}{ccc}
        0.25 & 0 & 0\\
        0 & 0.8 & 0\\
        0 & 0 & 0.33
        \end{array}
        \right]


    .. math::
        :label: eq:jordan_U

        U=\left[
        \begin{array}{ccc}
        1 & 0.75 & 0.25 \\
        0 & 1 & -1\\
        0 & 0 & 1
        \end{array}
        \right]


    :math:`LU` and :math:`LDU` factorization require the same number of operations; the primary difference lies in how the book–keeping is performed. The :math:`LDU` decomposition algorithm is often used in place of the :math:`LU` algorithm.



    Kinematic Substructuring compared to work of Kecksemethy
    ************************************************************

    Automatic Closed-Form Kinematics-Solutions for Recursive Single-Loop Chains
    ------------------------------------------------------------------------------------------
      * Analyzes each kinematic loop independently.
      * For each independent kinematic loop, a projection method is used to convert SE3 matrix equations to a *possibly* simpler form.

          * quadratic distance between two points
          * distance between a point and a plane
          * cosine of the angle between two planes
          * distance between two lines along common perpendicular
          * quadratic distance of a point to a line

      * To determine which projections to apply, Kecksemethy carries out cyclic permutations of transformations of loop members, applying different projection methods. If it is found that two transformations can be arranged adjacently that leave a geometric element invariant (i.e. the origin, coordinate planes or the coordinate axes), the two transformations can be replaced with a simpler transformation.


    On Closed Form Solutions of Multiple-Loop Mechanisms
    ----------------------------------------------------

      * Discusses concepts from "Automatic Closed-Form .."
      * Discusses minimum cycle basis method to generate Jacobian (i.e. an algorithm to find a set of independent kinematic loops with the least number of elements in each loop, thereby simplifying the constraint equations). The computational efficiency is roughly O(n^3). *Note, my method is based on a modified selection sort algorithm and has a run time of O(1/2 n^2), the resulting Jacobian is not a minimum cycle basis. Using the minimum cycle basis for the Jacobian may result in a increased efficiency.*
      * Performs an analysis on the loop matrix to determine how to set up solution flow.


    Symbolic Processing of Multiloop Mechanism Dynamics Using Closed-Form Solutions
    ---------------------------------------------------------------------------------------

      * The primary focus of the previous papers is on kinematics. This paper extends methods of previous three papers to include velocities, accelerations and dynamics.
      * A closed-form solution to a kinematic loop can be found if the loop can be reduced to 6 scalar equations using the cyclic permutation and projection method. Otherwise, an iterative method must be used to solve for the loop (they note that a 7DOF robot arm is an example where a closed-form solution does not exist.)


    Precomputed Loop-Based Kinematics and Dynamics in Large Scale Vehicle Models
    -----------------------------------------------------------------------------------

      * Create a simplified model of each kinematic loop individually, neglecting the rest of the system. This simplified model is referred to as a nonlinear *kinematical transformer*.
      * To set up the kinematics of the kinematics of the overall system, the kinematic transformers are arranged to feed into each other at summing junctions.
      * Paper does not describe how process is carried out, refers to "On Closed-Form Solutions of Multiple-Loop Mechanisms" for details.


    Sparse--Matrix Generation of Jacobians for the Object--Oriented Modeling of Multibody Systems
    ----------------------------------------------------------------------------------------------

      * Discusses using object--oriented programming to arrange multibody system data. Since my core solver is programmed in C, I use the functional programming paradigm.
      * Kecksemethy refers to *influence coefficient matrices* as "*local Jacobians*".
      * The primary difference between their sparse matrix representation of a Jacobian datastructure and general sparse matrix representation is that his *Jacobian object* has pointers to influence coefficient matrices for each joint. My Jacobian matrix also has pointers to each influence coefficient matrix, except that positive and loop members are stored separately. To maximize SIMD, I evaluate all positively oriented loop members, followed by all negatively oriented loop members, which may yield a minor improvement in speed.
      * The paper discusses aspects of concatenating Jacobians in complex mechanisms.
      * Kecksemethy compares the execution speed of two methods to evaluate the Jacobian matrix. They are based on a binary tree data structure, which requires the use of a temporary stack to traverse.
      * In the kinematic substructuring paper, I used the binary tree only to set the levels list. I have since revised the substructuring algorithm so that I generate the levels list without a need for the binary tree at all. The parent--child list (or automata programming method) is more efficient for traversing graph as it does not require a temporary stack.
