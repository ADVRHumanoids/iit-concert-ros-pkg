ros2(){
    local target script_dir env_file
    target="$1"

    if [ -z "$target" ]
    then
        echo "No argument supplied" >&2
        return 1
    fi

    script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    env_file="$script_dir/../concert-config_ros2.env"

    if [ ! -f "$env_file" ]; then
        echo "Concert docker environment file not found: $env_file" >&2
        return 1
    fi

    # shellcheck disable=SC1091
    source "$env_file"
    export CONCERT_ACTIVE_DOCKER_ENV=ros2

    cd "$script_dir" || return 1
    xhost +local:root
    echo "Running docker image from $PWD..."
    docker compose up "$target" -d --no-recreate
    docker compose exec "$target" bash
}