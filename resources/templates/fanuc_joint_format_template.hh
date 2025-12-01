inline static void format_to_robot_configuration(Configuration &q) noexcept
{
    std::array<float, 8> arr8 = q.to_array();
    std::array<float, 6> arr = {arr8[0], arr8[1], arr8[2], arr8[3], arr8[4], arr8[5]};
    arr[2] -= arr[1];
    q = Configuration(arr);
}

inline static void format_to_vamp_configuration(Configuration &q) noexcept
{
    std::array<float, 8> arr8 = q.to_array();
    std::array<float, 6> arr = {arr8[0], arr8[1], arr8[2], arr8[3], arr8[4], arr8[5]};
    arr[2] += arr[1];
    q = Configuration(arr);
}