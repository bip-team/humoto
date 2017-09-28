/**
    @file
    @author  Alexander Sherikov
    @author Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    //========================================================================
    // Shorthand types
    //========================================================================

    typedef Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>      IndexVector;

    /**
     * Index type used by Eigen. We should not use Eigen::DenseIndex since it
     * is going to be deprecated in new versions of Eigen. May be EigenIndex
     * should be defined depending on the version of Eigen.
     */
    typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE                      EigenIndex;


    //========================================================================
    // Generic types
    //========================================================================


    /**
     * @brief Index of an axis
     */
    class AxisIndex
    {
        public:
            enum Index
            {
                X = 0,
                Y = 1,
                Z = 2
            };

            enum Flag
            {
                FLAG_UNDEFINED = 0,
                //          Z   Y   X
                FLAG_X =            1,
                FLAG_Y =        2,
                FLAG_Z =    4
            };
    };


    /**
     * @brief Indices of RPY angles
     */
    class AngleIndex
    {
        public:
            enum Index
            {
                ROLL = 0,
                PITCH = 1,
                YAW = 2
            };
    };


    //========================================================================
    // Math
    //========================================================================

    /**
     * @brief Returns true if the difference between two given variables is
     * below the given tolerance.
     *
     * @param[in] var1
     * @param[in] var2
     * @param[in] tol
     *
     * @return true / false
     */
    inline bool isApproximatelyEqual(  const double var1,
                                const double var2,
                                const double tol = humoto::g_generic_tolerance)
    {
        return (std::abs(var1 - var2) < tol);
    }


    //========================================================================
    // Geometry
    //========================================================================


    /**
     * @brief Convert degrees to radians
     *
     * @param[in] degrees degrees
     *
     * @return radians
     */
    inline double HUMOTO_LOCAL convertDegreesToRadians(const double degrees)
    {
        return (humoto::g_pi * degrees / 180);
    }


    /**
     * @brief Convert radians to degrees
     *
     * @param[in] radians
     *
     * @return degrees
     */
    inline double HUMOTO_LOCAL convertRadiansToDegrees(const double radians)
    {
        return (radians * 180 / humoto::g_pi);
    }


    /**
     * @brief Function computing side length of a square
     *        inscribed inside of a circle of given radius.
     *
     * @param  radius circle radius
     * @return length of a side of a square inscribed in the circle
     */
    inline double HUMOTO_LOCAL getEncircledSquareSide(const double radius)
    {
        return((2. / std::sqrt(2.)) * radius);
    }


    //========================================================================
    // Simple generic classes
    //========================================================================

    /**
     * @brief Location of a data chunk (offset + length).
     */
    class HUMOTO_LOCAL Location
    {
        public:
            std::size_t    offset_;
            std::size_t    length_;


            /**
             * @brief Constructor
             */
            Location()
            {
                set(0, 0);
            }


            /**
             * @brief Construct location with given parameters
             *
             * @param[in] offset offset
             * @param[in] length length
             */
            Location (const std::size_t offset, const std::size_t length)
            {
                set(offset, length);
            }


            /**
             * @brief Compare two locations
             *
             * @param[in] another_location some other location
             *
             * @return true/false
             */
            bool operator== (const Location &another_location) const
            {
                return ( (offset_ == another_location.offset_) &&
                        (length_ == another_location.length_) );
            }


            /**
             * @brief Compare two locations
             *
             * @param[in] another_location some other location
             *
             * @return true/false
             */
            bool operator!= (const Location &another_location) const
            {
                return ( !(*this == another_location) );
            }


            /**
             * @brief Check length of location
             *
             * @param[in] max_length maximal length
             *
             * @return true if location is valid
             */
            bool checkLength(const std::size_t max_length) const
            {
                return (    (offset_ <= end()) &&
                            (end() <= max_length) );
            }


            /**
             * @brief Offset of the first element
             *
             * @return offset of the first element (offset_)
             */
            std::size_t front() const
            {
                return(offset_);
            }


            /**
             * @brief Offset of the element following the last element
             *
             * @return offset of the element following the last element (offset_ + length_)
             */
            std::size_t end() const
            {
                return(offset_ + length_);
            }


            /**
             * @brief Set location.
             *
             * @param[in] offset offset
             * @param[in] length length
             */
            void set(const std::size_t offset, const std::size_t length)
            {
                offset_ = offset;
                length_ = length;
            }


            /**
             * @brief Log hierarchy as a set of tasks
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            void    log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                           const LogEntryName & parent = LogEntryName(),
                           const std::string & name = "location") const
            {
                LogEntryName subname = parent;
                subname.add(name);

                logger.log(LogEntryName(subname).add("offset"), offset_);
                logger.log(LogEntryName(subname).add("length"), length_);
            }
    };
}
