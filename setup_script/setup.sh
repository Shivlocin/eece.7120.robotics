#!/bin/bash

set -u
my_loc=$(pwd)

# Find the name of this script
me=${0##*/}

# This script builds and configures an environment for the fundamentals of 
# robotics class.

# The variables in this block can be changed to suit the users preference

project_name="robotics_class"
project_name_c="Robotics Class"

# The variables below should be altered appropriately once the functionality
# requiring them is implemented

project_repo="insert repo URL here"
project_repo_branch="master"

# The variables below here should not be altered unless you know what you're 
# doing.
local_host_mount=$HOME/.${project_name}
config_scripts=false
sandbox=${my_loc}/sandbox
project_home=$sandbox/$project_name
catkin_ws=${my_loc}/catkin_ws
user=student
home=/home/$user
rebuild=false
isolate=false
cmd="/bin/bash"

# This flag blocks off some code to run internal helper scripts which are 
# currently not required or supported. If, in the future, you wish to use
# this feature with a git-repo containing helper scripts remove the blocking
# conditional ("BLOCKING CONDITIONAL") and alter as appropriate,
config_scripts=false
# Location of the setup script INSIDE OF THE CONTAINER
setup_script=""


# CLI help and documentation
options="Available options are:\n"
options="$options\t -r | --rebuild Rebuild the container.\n"
options="$options\t -x | --execute Execute the given command in the container.\n"
options="$options\t -i | --isolate Isolate the containers network.\n"
options="$options\t -b | --branch  Use the specified branch to build the env.\n"

clihelp="Basic information about this script\n"
clihelp="$clihelp\t This script constructs a universal environment to aid\n" 
clihelp="$clihelp\t students by ensuring their development environment\n"
clihelp="$clihelp\t for the class is built and configured properly.\n\n"

clihelp="$clihelp\t This degree of consistency also simplifies troubleshooting\t"
clihelp="$clihelp\t by minimizing environmental inconsistencies such as an\n"
clihelp="$clihelp\t an improperly configured network stack or broken toolchain\n"



function checkOS {
  
  # Sanity check, make sure the operating system is Ubuntu
  if [ -f /etc/lsb-release -o -d /etc/lsb-release.d ]; then
      export DISTRO=$(lsb_release -i | cut -d: -f2 | sed s/'^\t'//)
      if [[ "Ubuntu" != $DISTRO ]]; then
          echo "The Operating System must be Ubuntu to use this script."
          exit
      fi
  else
      echo "Cannot determine the Operating System. lsb-release is not installed"
      exit
  fi
}


function installDocker {
  docker_pkg="docker.io"
  has_docker=$(dpkg-query -W --showformat='${Status}\n' $docker_pkg | grep "install ok installed")
  if [[ -z $has_docker ]]; then
    echo
    echo -e "\e[34mInstalling Docker.io\e[0m"
    echo
    sudo apt update
    sudo apt install --force-yes --yes \
      docker.io                        \
      docker-compose                   \

   sudo groupadd docker
   sudo usermod -a -G docker $USER
   sudo systemctl enable docker
   sudo dockerd
    
    if  ! id -nG "$USER" | grep -qw "docker"; then
      # Docker group hasn't taken effect yet. Force it here.
      echo "You are not registered for the docker group yet. You should log out"
      echo "and log in again so it takes effect."
      exec su -l $USER
    fi
  fi      
}


while [ $# -gt 0 ];
do
  key="$1"

  case $key in 
    -r|--rebuild)
      rebuild=true
      shift
      ;;
    -x|--execute)
      cmd="$2"
      shift
      shift
      ;;
    -i|--isolate)
      isolate=true
      shift
      ;;
    -h|--help)
      echo $clihelp
      echo $options
      shift
      ;;
    -b|--branch)
      project_repo_branch="$2"
      shift
      shift
      ;;
    *)
      echo $options
      exit 1;
      shift
      ;;
  esac
done


# Check the operating system before proceeeding. If its not Ubuntu stop.
checkOS

# Install the required directories.
install -d "$sandbox"
install -d "$catkin_ws"

# Install the required tools.
installDocker


#########################
####                 ####
#### Build the image ####
####                 ####
#########################
                    
                    ###################
                    ###             ###
                    ###  Non-cross  ###
                    ###             ###
                    ###################
# Check if a millennium image (non cross) already exists
if [[ 2 -gt $(docker images $project_name | wc -l) ]]; then
    # No image, build one  
    echo
    echo -e "\e[34mBuilding The ${project_name_c} Docker.\e[39m"
    echo
    docker build --build-arg UID=$UID -t ${project_name}:base ${my_loc}/docker/
    
    # Provide a convenience executable for opening a shell
    if [[ ! -d "$HOME/bin" ]]; then
        mkdir "$HOME/bin"
        echo "No bin directory, creating one. You may need to log out before "\
            "the directory is added to your path."
    fi
    echo "pushd \"$my_loc\" > /dev/null; bash ./${0##*/} \"\$@\"; popd > /dev/null;" > "$HOME/bin/${project_name}"
    sudo chmod +x "$HOME/bin/${project_name}"

    if [[ ! -d "$local_host_mount" ]]; then
      install -d $local_host_mount
      chown $USER:$USER -R $local_host_mount
    fi


    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    if $config_scripts; then
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    
    # Check to see if we have the sources
    if [[ -d $project_home/.git ]]; then
        # We have the sources. Make sure we have the right version.
        pushd $project_home
        current_branch=$(git symbolic-ref --short HEAD)
        if [[ $current_branch != $project_repo_branch ]]; then
            echo
            echo -e "\e[34mChecking out the $project_repo_branch branch.\e[39m"
            echo
            git checkout $project_repo_branch || { echo "Failed to checkout the desired branch"; exit 1; }
        fi
        popd
    else
        # We do not have the sources, get them.
        echo
        echo -e "\e[34mCloning The ${project_name_c} Code.\e[39m"
        echo
        ## Check out the endor repository into the sandbox
        git clone --recurse-submodules -b $project_repo_branch $project_repo $project_home
    fi
    # Run the setup scripts inside the container
    echo
    echo -e "\e[34mRunning the configuration scripts.\e[39m"
    echo
    
    # Create the catkin_ws directory prior to mounting it to a container. This
    # prevents the directory from being owned by root.

    docker run                                                              \
        --cidfile "${my_loc}/id.cid"                                        \
        --net=host                                                          \
        -t                                                                  \
        -v "${sandbox}:$home/sandbox"                                       \
        -v "${catkin_ws}:$home/sandbox/catkin_ws" \
        -v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK):ro           \
        -v $HOME/.ssh:$home/.ssh                                            \
        -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK                                     \
        --user $user                                                        \
        -w $home/sandbox/endor/config/scripts                               \
        -i "${project_name}:base"                                         \
        /bin/bash -ci "$setup_script"
 


    # If the setup was successful, commit the result as a new image    
    result="$?"
    cid=`cat ${my_loc}/id.cid`
    rm -f "${my_loc}/id.cid"
    [[ "$result" -eq "0" ]] || { 
        echo "Failed to create base docker image."; 
        docker rmi -f ${project_name}:base;
        exit 1; 
    }
    docker commit $cid "${project_name}:latest"
    docker stop $cid && docker rm $cid
    docker rmi "${project_name}:base"

    # Provide a convenience executable for opening a shell
    if [[ ! -d "$HOME/bin" ]]; then
        mkdir "$HOME/bin"
        echo "No bin directory, creating one. You may need to log out before "\
            "the directory is added to your path."
    fi
    echo "pushd \"$my_loc\" > /dev/null; bash ./${0##*/} \"\$@\"; popd > /dev/null;" > "$HOME/bin/${project_name}"
    sudo chmod +x "$HOME/bin/${project_name}"

    # Miscellaneous saved data
    [[ -d $local_host_mount ]] || mkdir $local_host_mount
    touch "$local_host_mount/bash_history"
    echo "$project_repo_branch" > "$local_host_mount/git_branch.txt" 

    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
  fi
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
    ####### BLOCKING CONDITIONAL ########
fi

                        ################
                        ###          ###
                        ###  Launch  ###
                        ###          ###
                        ################
echo
echo -e "\e[34mStarting The ${project_name_c} Docker.\e[39m"
cat << "EOF"
                                                      ____
                                                     /    `.
                                                    /-----.|          ____
                                                ___/___.---`--.__.---'    `--.
                                  _______.-----'           __.--'             )
                              ,--'---.______________..----'(  __         __.-'
                                        `---.___,-.|(a (a) /-'  )___.---'
                                                `-.>------<__.-'
            ______                       _____..--'      //
    __.----'      `---._                `._.--._______.-'/))
,--'---.__              -_                  _.-(`-.____.'// \
          `-._            `---.________.---'    >\      /<   \
              \_             `--.___            \ \-__-/ /    \
                \_                  `----._______\ \  / /__    \
                  \                      /  |,-------'-'\  `-.__\
                   \                    (   ||            \      )
                    `\                   \  ||            /\    /
                      \                   >-||  @)    @) /\    /
                      \                  ((_||           \ \_.'|
                       \                    ||            `-'  |
                       \                    ||             /   |
                        \                   ||            (   '|
                        \                   ||  @)     @)  \   |
                         \                  ||              \  )
                          `\_               `|__         ____\ |
                             \_               | ``----'''     \|
                               \_              \    .--___    |)
                                 `-.__          \   |     \   |
                                      `----.___  \^/|      \/\|
                                               `--\ \-._  / | |   
                                                   \ \  `'  \ \
                                            __...--'  )     (  `-._
                                           (_        /       `.    `-.__
                                             `--.__.'          `.       )
                                                                 `.__.-'
EOF

docker_cmd=docker
network="--net=host"
mount_tmp="-v /tmp:/tmp"

if $isolate; then
  network=""
  mount_tmp=""
fi

$docker_cmd run                                                         \
    --rm                                                                \
    $network                                                            \
    -t                                                                  \
    --privileged                                                        \
    --cap-add=SYS_PTRACE --security-opt seccomp=unconfined              \
    -v $local_host_mount/bash_history:$home/.bash_history               \
    -v $local_host_mount:$home/host                                      \
    -v /dev/bus/usb:/dev/bus/usb                                        \
    -v "${sandbox}:$home/sandbox"                                       \
    -v "${catkin_ws}:$home/sandbox/catkin_ws"                           \
    -v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK):ro           \
    -v $HOME/.ssh:$home/.ssh                                            \
    -v /tmp/.X11-unix:/tmp/.X11-unix                                    \
    $mount_tmp                                                          \
    -e DISPLAY=unix$DISPLAY                                             \
    -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK                                     \
    --user $user                                                        \
    -w $home                                                            \
    -i "${project_name}:base"                                              \
    /bin/bash -ci "${cmd}"
