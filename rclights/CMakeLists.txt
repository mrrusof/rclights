add_executable(rclights
        rclights.c
        )

# pull in common dependencies
target_link_libraries(rclights pico_stdlib hardware_pwm)

# create map/bin/hex file etc.
pico_add_extra_outputs(rclights)

# add url via pico_set_program_url
example_auto_set_url(rclights)
