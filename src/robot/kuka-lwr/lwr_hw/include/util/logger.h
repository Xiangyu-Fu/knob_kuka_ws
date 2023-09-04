#ifndef LWR_HW__LOGGER_H
#define LWR_HW__LOGGER_H

#include <vector>
#include <string>

namespace lwr_hw {

/**
 * @brief Logger Class
 * 
 * Creates a histogram of elapsed time of the control loop.
 * 
 * Usage:
 *  uint64_t dt = (ros::Time::now() - prev_time).toNSec() / 1000;
 *  logger.update(dt)
 *  
 *  logger.flush("hist.txt")
 * 
 */
class Logger
{
private:
  uint64_t hist_size_;
  std::vector<int> hist_;
  std::vector<uint64_t> outliers_;

  uint64_t max_;
  uint64_t min_;
  double avg_;
  int cnt_;
  int warmup_cnt_;
  int overflow_cnt_;

public:
  /**
   * @brief Construct a new Logger object
   * 
   * @param hist_size corespondes to maximum log time 100000 = 100 ms
   */
  Logger(uint64_t hist_size = 100000, int warmup_cnt=500) : 
    hist_size_(hist_size),
    max_(0),
    min_(hist_size),
    avg_(0),
    cnt_(0),
    warmup_cnt_(warmup_cnt),
    overflow_cnt_(0)
  {
    hist_.resize(hist_size_, 0);
    outliers_.resize(16384, 0);
  }

  ~Logger()
  {
  }

  void update(uint64_t time_us)
  {
    if (cnt_ > warmup_cnt_)
    {
      if (time_us > hist_size_)
      {
        overflow_cnt_ += 1;
      }
      else 
      {
        hist_[time_us]++;
      }

      if (time_us < min_)
      {
        min_ = time_us;
      }
      if (time_us > max_)
      {
        max_ = time_us;
      }

      avg_ = avg_ + (double(time_us) - avg_)/(cnt_ - warmup_cnt_);
    }
    cnt_++;
  }

  void flush(const std::string& file)
  {
    if(cnt_ <= 1)
    {
      return;
    }

    FILE* fp;
    fp = fopen(file.c_str(), "w");
    if(!fp)
    {
      return;
    }
    for(int i = 0; i < hist_size_; ++i)
    {
      fprintf(fp, "%06d %06d\n", i, hist_[i]);
    }
    fclose(fp);
  }

  void print()
  {
    printf("n=\t%d\n", cnt_ - warmup_cnt_);
    printf("n_ow=\t%d\n", overflow_cnt_);
    printf("min=\t%lu us\n", min_);
    printf("max=\t%lu us\n", max_);
    printf("avg=\t%f us\n", avg_);
    printf("freq=\t%f Hz\n", 1./avg_*1e6);
  }
};

}

#endif