/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace humoto
{
    /**
     * @brief Converts milliseconds to seconds
     *
     * @param[in] milliseconds milliseconds
     *
     * @return seconds.
     */
    inline double HUMOTO_LOCAL convertMillisecondToSecond(const std::size_t milliseconds)
    {
        return (milliseconds / 1000.0);
    }


    /**
     * @brief Converts seconds to milliseconds
     *
     * @param[in] seconds seconds
     *
     * @return milliseconds.
     */
    inline std::size_t HUMOTO_LOCAL convertSecondToMillisecond(const double seconds)
    {
        return (  static_cast<std::size_t>( round(seconds * 1000) )  );
    }


    /**
     * @brief Converts seconds to milliseconds (with truncation instead of rounding)
     *
     * @param[in] seconds seconds
     *
     * @return milliseconds.
     */
    inline std::size_t HUMOTO_LOCAL truncateSecondToMillisecond(const double seconds)
    {
        return (  static_cast<std::size_t>( std::floor(seconds * 1000) )  );
    }


    /**
     * @brief Timer
     */
    class HUMOTO_LOCAL Timer
    {
        public:
            /**
             * @brief Constructor. Starts the timer.
             */
            Timer()
            {
                timediff = 0.0;
                start();
            }


            /**
             * @brief Re/starts the timer.
             */
            void start()
            {
                gettimeofday(&start_time, 0);
            }


            /**
             * @brief Stops the timer.
             */
            double stop()
            {
                gettimeofday(&end_time, 0);
                timediff = (double) end_time.tv_sec - start_time.tv_sec
                         + 0.000001 * (end_time.tv_usec - start_time.tv_usec);
                return(timediff);
            }


            /**
             * @brief Returns the measured time interval (second).
             *
             * @return time interval (second).
             */
            double get() const
            {
                return(timediff);
            }


            /**
             * @brief Outputs the measured time interval.
             *
             * @param[in,out] out output stream.
             * @param[in] timer the timer.
             *
             * @return output stream.
             */
            friend std::ostream& operator<< (std::ostream& out, const Timer& timer)
            {
                std::ios  state(NULL);
                state.copyfmt(out);

                out << std::setiosflags(std::ios::fixed) << std::setprecision(6)
                    << "Timer value = "        << timer.timediff;

                out.copyfmt(state);
                return(out);
            }


        private:
            struct timeval start_time;
            struct timeval end_time;
            double timediff;
    };
}
