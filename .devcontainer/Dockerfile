################################################################################
# Base
#  - https://hub.docker.com/_/ros/
################################################################################

ARG ROS_DISTRO=foxy

FROM ros:${ROS_DISTRO}

ARG NAME=ros-${ROS_DISTRO}

################################################################################
# Updates
################################################################################

# TODO: Delete the list of debs. These are merely the list
# required by rosdep for the py-trees set of repos on foxy
# and this would be better handled by rosdep itself
# so it can gracefully transition across distros.
#
# NB: Installing debs here for now to expedite testing of the
# devcontainer configuration (esp. for qt, X11).
#
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Docs
    python3-venv
    # PyTrees
    # ros-${ROS_DISTRO}-py-trees
    # Dot
    # python3-pydot \
    # Qt
    # libqt5webkit5 \
    # pyqt5-dev-tools \
    # python3-pyqt5 \
    # Other
    # qttools5-dev-tools \
    # libpyside2-dev \
    # libshiboken2-dev \
    # pyqt5-dev \
    # python3-pyqt5.qtsvg \
    # python3-pyside2.qtsvg \
    # python3-sip-dev \
    # shiboken2 \
    # python3-pyqt5.qtwebengine \
    # python3-mypy

################################################################################
# Login Shells for Debugging & Development
################################################################################

# In a login shell (below), the PATH env doesn't survive, configure it at ground zero
# RUN echo "export PATH=${POETRY_HOME}/bin:${PATH}" >> /etc/profile
ENV TERM xterm-256color
ENTRYPOINT ["/bin/bash", "--login", "-i"]

################################################################################
# Development with a user, e.g. for vscode devcontainers
################################################################################

ARG USERNAME=zen
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -s "/bin/bash" -m $USERNAME && \
    apt-get install -y sudo && \
    echo "${USERNAME} ALL=NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}
RUN echo "export PS1='\[\033[01;36m\](docker)\[\033[00m\] \[\033[01;32m\]\u@${NAME}\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /home/${USERNAME}/.bashrc && \
    echo "alias ll='ls --color=auto -alFNh'" >> /home/${USERNAME}/.bashrc && \
    echo "alias ls='ls --color=auto -Nh'" >> /home/${USERNAME}/.bashrc

USER zen
RUN rosdep update
USER root

################################################################################
# Debugging with root
################################################################################

RUN echo "export PS1='\[\033[01;36m\](docker)\[\033[00m\] \[\033[01;32m\]\u@${NAME}\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ${HOME}/.bashrc && \
    echo "alias ll='ls --color=auto -alFNh'" >> ${HOME}/.bashrc && \
    echo "alias ls='ls --color=auto -Nh'" >> ${HOME}/.bashrc
