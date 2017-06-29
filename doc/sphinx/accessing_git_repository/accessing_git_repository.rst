.. title:: Accessing My Git Repository

Accessing My Git Repository
===========================

My PhD research (source code, publications, references and documentation) is stored in a git repository which is accessible by ssh at this web address.

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

        HostName dhcp223.ahmct.ucdavis.edu
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
