#!/usr/bin/env bash
# =============================================================================
# AnimClock : Main Control Script
# =============================================================================

#set -euo pipefail

# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------
readonly INSTALL_DIR="/opt/AnimClock"
readonly DATA_DIR="/media/data/AnimClock"
readonly CONFIG_FILE="$DATA_DIR/config/config.txt"
readonly FONT_DIR="$DATA_DIR/fonts"
readonly GIF_DIR="$DATA_DIR/gif"
readonly WEATHER_IMAGE="/tmp/weather.png"

# -----------------------------------------------------------------------------
# Default Panel Configuration
# -----------------------------------------------------------------------------
Panel_XSize="128"
Panel_YSize="32"

# -----------------------------------------------------------------------------
# Default Clock Configuration
# -----------------------------------------------------------------------------
Clock_Active="1"
Clock_Display_Time="3000"
Clock_TZ="Europe/Paris"
Clock_Format="%H:%M:%S"
Clock_Xpos="8"
Clock_Ypos="8"
Clock_Font="GOUDYSTO.TTF"
Clock_Font_Size="11"
Clock_Alignement="4"
Clock_Font_Pattern="Pattern_texte2.png"
Clock_Background=""

# -----------------------------------------------------------------------------
# Default Date Configuration
# -----------------------------------------------------------------------------
Date_Active="1"
Date_Format="%d %b"
Date_Display_Time="2000"
Date_Xpos="5"
Date_Ypos="8"
Date_Font="GOUDYSTO.TTF"
Date_Font_Size="9"
Date_Alignement="4"
Date_Font_Pattern="Pattern_texte3.png"
Date_Background=""
Date_Locale="fr_FR"

# -----------------------------------------------------------------------------
# Default Weather / GIF Configuration
# -----------------------------------------------------------------------------
OpenWeather_ID=""
OpenWeather_Country=""
OpenWeather_ZipCode=""
Weather_Text_color="255,255,0"
Weather_Display_Time="3000"
Weather_Active="1"

Gif_Active="1"

# -----------------------------------------------------------------------------
# Runtime State
# -----------------------------------------------------------------------------
md5_old_config=""
current_hour=$(date +%H | sed 's/^0*//')
prev_hour="xx"
prev_ip_eth0=""
prev_ip_wlan0=""

# =============================================================================
# FUNCTIONS
# =============================================================================

# -----------------------------------------------------------------------------
# load_config : Read parameters from config file and apply them
# -----------------------------------------------------------------------------
load_config() {
    local param_list=(
        Panel_XSize Panel_YSize 
        Clock_Format Clock_TZ Clock_Display_Time
        Clock_Xpos Clock_Ypos Clock_Font Clock_Font_Size
        Clock_Alignement Clock_Font_Pattern Clock_Background
        Date_Format Date_Display_Time
        Date_Xpos Date_Ypos Date_Font Date_Font_Size
        Date_Alignement Date_Font_Pattern Date_Background
        OpenWeather_ID OpenWeather_Country OpenWeather_ZipCode
        Weather_Text_color Weather_Display_Time
        Gif_Active Clock_Active Date_Active Weather_Active
        Date_Locale
    )

    for param in "${param_list[@]}"; do
        local value
        value=$(grep "^${param}=" "$CONFIG_FILE" | cut -d'=' -f2)
        eval "$param='$value'"
    done

    # Build the list of search directories for GIF files
    unset gif_search_dirs_
    local idx=1
    while IFS= read -r category; do
        local dir="$GIF_DIR/$category"
        if [[ -d "$dir" ]]; then
            gif_search_dirs_[$idx]="$dir"
            (( idx++ ))
        fi
    done < <(grep "^Gif_Cat" "$CONFIG_FILE" | grep -v "disabled" | cut -d'=' -f2)

    # Fall back to root GIF directory when no categories are defined
    if [[ $idx -eq 1 ]]; then
        gif_search_dirs_[1]="$GIF_DIR"
    fi

    # Sync timezone if it differs from the configured one
    local current_tz
    current_tz=$(timedatectl | grep "Time zone" | awk '{print $3}')
    if [[ "$current_tz" != "$Clock_TZ" ]]; then
        sudo timedatectl set-timezone "$Clock_TZ"
    fi
}

# -----------------------------------------------------------------------------
# check_config : Reload config only when the file has changed (MD5 check)
# -----------------------------------------------------------------------------
check_config() {
    local new_md5
    new_md5=$(md5sum "$CONFIG_FILE" | cut -d' ' -f1)
    if [[ "$new_md5" != "$md5_old_config" ]]; then
        load_config
        md5_old_config="$new_md5"
    fi
}

# -----------------------------------------------------------------------------
# resolve_background PATTERN : picks one matching file from FONT_DIR
#   $1 : raw config value ("Random", "None", a filename, or a glob pattern)
#   Prints the resolved absolute path, or nothing if disabled / not found.
# -----------------------------------------------------------------------------
resolve_background() {
    local pattern="$1"
    case "$pattern" in
        "None"|"")   return ;;                    # disabled
        "Random")    pattern="Background_*.gif" ;;
    esac

    # Expand glob without quoting so the shell resolves wildcards
    shopt -s nullglob
    local -a matches=( $FONT_DIR/$pattern )
    shopt -u nullglob

    if [[ ${#matches[@]} -gt 0 ]]; then
        printf '%s\n' "${matches[@]}" | shuf | tail -1
    fi
}

# -----------------------------------------------------------------------------
# update_weather_and_ip : Fetch weather once per hour; display IP on change
# -----------------------------------------------------------------------------
update_weather_and_ip() {
    current_hour=$(date +%H | sed 's/^0*//')

    # Refresh weather data at the start of every new hour
    if [[ "$current_hour" != "$prev_hour" && -n "$OpenWeather_ID" ]]; then
        local screen_w=$(( Panel_XSize ))
        local screen_h=$(( Panel_YSize ))
        "$INSTALL_DIR/Weather" \
            -i "$OpenWeather_ID" \
            -z "$OpenWeather_ZipCode" \
            -c "$OpenWeather_Country" \
            -X "$screen_w" \
            -Y "$screen_h" \
            -C "$Weather_Text_color" \
            -F "$WEATHER_IMAGE"
    fi
    prev_hour="$current_hour"

    # Re-display IP address if network configuration has changed
    local new_ip_eth0 new_ip_wlan0
    new_ip_eth0=$(ifconfig eth0  | grep "inet " || true)
    new_ip_wlan0=$(ifconfig wlan0 | grep "inet " || true)

    if [[ "$new_ip_eth0" != "$prev_ip_eth0" || "$new_ip_wlan0" != "$prev_ip_wlan0" ]]; then
        sudo "$INSTALL_DIR/Show_IP" \
            --led-cols="$Panel_XSize" \
            --led-rows="$Panel_YSize"
        #sudo hwclock -w
    fi

    prev_ip_eth0="$new_ip_eth0"
    prev_ip_wlan0="$new_ip_wlan0"
}

# -----------------------------------------------------------------------------
# get_common_led_flags : Shared --led-* flag set used by multiple binaries
# -----------------------------------------------------------------------------
get_common_led_flags() {
    echo \
        --led-cols="$Panel_XSize" \
        --led-rows="$Panel_YSize"
}

# =============================================================================
# MAIN
# =============================================================================
main() {

    cd "$INSTALL_DIR"

    check_config

    # Show boot animation
    sudo ./Boot $(get_common_led_flags)

    # -------------------------------------------------------------------------
    # Main display loop
    # -------------------------------------------------------------------------
    while true; do
        # Collect all .gif files from the configured directories, shuffle them
        mapfile -t gif_files < <(find "${gif_search_dirs_[@]}" -maxdepth 1 -name "*.gif" 2>/dev/null | shuf)

        if [[ ${#gif_files[@]} -eq 0 ]]; then
            echo "Warning: no GIF files found in ${gif_search_dirs_[*]}" >&2
            sleep 5
            continue
        fi

        for file in "${gif_files[@]}"; do

            update_weather_and_ip
            check_config

            # --- Weather panel ---
            if [[ -f "$WEATHER_IMAGE" && "$Weather_Display_Time" != "0" && "$Weather_Active" == "1" ]]; then
                sudo ./Show_Image -C $(get_common_led_flags) \
                    -T "$Weather_Display_Time" \
                    -F "$WEATHER_IMAGE"
            fi

            # --- Resolve randomised font / background paths ---
            local clock_font clock_bg clock_pattern clock_time
            local date_font  date_bg  date_pattern  date_time
            local gif_option

            # Fonts and patterns: simple glob expansion (no "Random"/"None" logic)
            shopt -s nullglob
            local -a _m
            _m=( $FONT_DIR/$Clock_Font );         clock_font=$(printf '%s\n' "${_m[@]}" | shuf | tail -1)
            _m=( $FONT_DIR/$Clock_Font_Pattern ); clock_pattern=$(printf '%s\n' "${_m[@]}" | shuf | tail -1)
            _m=( $FONT_DIR/$Date_Font );          date_font=$(printf '%s\n' "${_m[@]}" | shuf | tail -1)
            _m=( $FONT_DIR/$Date_Font_Pattern );  date_pattern=$(printf '%s\n' "${_m[@]}" | shuf | tail -1)
            shopt -u nullglob

            # Backgrounds: handles "Random", "None", and explicit filenames/globs
            clock_bg=$(resolve_background "$Clock_Background")
            date_bg=$(resolve_background  "$Date_Background")

            clock_time="$Clock_Display_Time"
            date_time="$Date_Display_Time"

            gif_option="-g $file"

            # Disable clock / date / GIF according to config flags
            [[ "$Clock_Active"   == "0" ]] && clock_time="0"
            [[ "$Date_Active"    == "0" ]] && date_time="0"

            if [[ "$Gif_Active" == "0" ]]; then
                gif_option=""

                # When only one module is active, use its background animation
                # duration as the display time (fallback: 30 minutes)
                if [[ "$Clock_Active" == "1" && "$Date_Active" == "0" && "$Weather_Active" == "0" ]]; then
                    local anim_delay
                    anim_delay=$("$INSTALL_DIR/Get_GIF_delay" -F "$clock_bg" | grep "^Delay=" | cut -d'=' -f2)
                    clock_time=$( [[ -z "$anim_delay" || "$anim_delay" == "0" ]] && echo "1800000" || echo "$anim_delay" )
                fi

                if [[ "$Date_Active" == "1" && "$Clock_Active" == "0" && "$Weather_Active" == "0" ]]; then
                    local anim_delay
                    anim_delay=$("$INSTALL_DIR/Get_GIF_delay" -F "$date_bg" | grep "^Delay=" | cut -d'=' -f2)
                    date_time=$( [[ -z "$anim_delay" || "$anim_delay" == "0" ]] && echo "1800000" || echo "$anim_delay" )
                fi
            fi

            # --- Run the animation ---
            sudo ./Anim_clock \
                --led-cols="$Panel_XSize" \
                --led-rows="$Panel_YSize" \
                $gif_option \
                -T "$Clock_Format"       -t "$clock_time"    \
                -x "$Clock_Xpos"         -y "$Clock_Ypos"    \
                -f "$clock_font"         -s "$Clock_Font_Size" \
                -p "$clock_pattern"      -b "$clock_bg"      \
                -a "$Clock_Alignement"   \
                -D "$Date_Format"        -d "$date_time"     \
                -X "$Date_Xpos"          -Y "$Date_Ypos"     \
                -F "$date_font"          -S "$Date_Font_Size" \
                -P "$date_pattern"       -B "$date_bg"       \
                -A "$Date_Alignement"    \
                -L "$Date_Locale"        \
                -C 255,255,0
        done
    done
}

main "$@"
