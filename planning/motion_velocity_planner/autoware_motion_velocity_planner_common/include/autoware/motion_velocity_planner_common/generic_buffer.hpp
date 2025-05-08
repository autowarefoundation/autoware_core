#include <chrono>
#include <optional>
#include <vector>

template <typename ItemType>
class GenericBuffer
{
public:
  using TimePoint = std::chrono::system_clock::time_point;
  using Buffer = std::vector<ItemType>;

  virtual ~GenericBuffer() = default;

  virtual std::optional<ItemType> getNearestActiveItem() const = 0;

protected:
  Buffer buffer_;
  double min_off_duration_;
  double min_on_duration_;

  GenericBuffer() = default;

  GenericBuffer(double min_off_duration, double min_on_duration)
  : buffer_(), min_off_duration_(min_off_duration), min_on_duration_(min_on_duration)
  {
  }
}
