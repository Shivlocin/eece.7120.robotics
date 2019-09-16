#!/bin/bash

set -u
my_loc=$(pwd)

# This script builds and configures an environment for the fundamentals of 
# robotics class.

# The variables in this block can be changed to suit the users preference

project_name=robotics_class
project_name_c="Robotics Class"


# The variables below here should not be altered unless you know what you're 
# doing.
local_host_mount=$HOME/.${project_name}
config_scripts=false
sandbox=${my_loc}/sandbox
catkin_ws=${my_loc}/catkin_ws
user=ubuntu
home=/home/$user
rebuild=false
isolate=false
cmd="/bin/bash"

# CLI help and documentation
options="Available options are:\n"
options="$options\t -r | --rebuild Rebuild the container.\n"
options="$options\t -x | --execute Execute the given command in the container.\n"
options="$options\t -i | --isolate Isolate the containers network.\n"

clihelp="Basic information about this script\n"
clihelp="$clihelp\t This script constructs a universal environment to aid\n" 
clihelp="$clihelp\t students by ensuring their development environment\n"
clihelp="$clihelp\t for the class is built and configured properly.\n\n"

clihelp="$clihelp\t This degree of consistency also aids with troubleshooting\t"
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
  has_docker=$(dpkg-quert -W --showformat='${Status}\n' $docker_pkg | grep "install ok installed")
  if [ -z $has_docker ]; then
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

    if
    # Run the setup scripts inside the container
    echo
    echo -e "\e[34mRunning the configuration scripts.\e[39m"
    echo
    
    # Create the catkin_ws directory prior to mounting it to a container. This
    # prevents the directory from being owned by root.
    if [ ! -d "${sandbox}/catkin_ws" ]; then
        install -d $sandbox/catkin_ws
    fi

    docker run                                                              \
        --cidfile "${my_loc}/id.cid"                                        \
        --net=host                                                          \
        -t                                                                  \
        -v "${sandbox}:$home/sandbox"                                       \
        -v "${ws_sandbox_loc}/catkin_ws.${target_arch}:$home/sandbox/catkin_ws" \
        -v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK):ro           \
        -v $HOME/.ssh:$home/.ssh                                            \
        -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK                                     \
        --user $user                                                        \
        -w $home/sandbox/endor/config/scripts                               \
        -i "${code_base_name}:base"                                         \
        /bin/bash -ci "$setup_endor"
 


    # If the setup was successful, commit the result as a new image    
    result="$?"
    cid=`cat ${my_loc}/id.cid`
    rm -f "${my_loc}/id.cid"
    [[ "$result" -eq "0" ]] || { 
        echo "Failed to create base docker image."; 
        docker rmi -f millennium:base;
        exit 1; 
    }
    docker commit $cid "${code_base_name}:latest"
    docker stop $cid && docker rm $cid
    docker rmi "${code_base_name}:base"

    # Provide a convenience executable for opening a millennium shell
    if [[ ! -d "$HOME/bin" ]]; then
        mkdir "$HOME/bin"
        echo "No bin directory, creating one. You may need to log out before "\
            "the directory is added to your path."
    fi
    echo "pushd \"$my_loc\" > /dev/null; bash ./millennium \"\$@\"; popd > /dev/null;" > "$HOME/bin/millennium"
    sudo chmod +x "$HOME/bin/millennium"

    # Miscellaneous saved data
    [[ -d $millennium_dir ]] || mkdir $millennium_dir
    touch "$millennium_dir/bash_history"
    echo "$endor_repo_branch" > "$millennium_dir/git_branch.txt" 

    if [[ "$dontCompile" = true ]]; then
        # Warn the user that the millennium script did not compile the 
        # codebase as part of the build process. 
        echo
        echo
        echo -e "\e[5m\e[1m\e[31m  WARNING! WARNING! WARNING! WARNING! \e[0m"
        echo
        echo -e "\e[1m    Millennium source code not compiled!"
        echo -e "   You must run catkin_make in catkin_ws "
        echo -e " and start millennium with the -isf argument"
        echo -e "   to finish setting up the environment!\e[0m"
        echo
        echo -e "\e[5m\e[1m\e[31m  WARNING! WARNING! WARNING! WARNING! \e[0m"
    fi
  fi
