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
     * @brief This enum is used to handle symmetric objects (left / right foot etc).
     */
    class LeftOrRight
    {
        public:
            enum Type
            {
                UNDEFINED = 0,
                LEFT = 1,
                RIGHT = 2,
                COUNT = 2
            };


        public:
            /**
             * @brief Exchange left and right
             *
             * @param[in] left_or_right LEFT or RIGHT
             *
             * @return RIGHT or LEFT
             */
            static Type invert(const Type left_or_right)
            {
                switch(left_or_right)
                {
                    case LEFT:
                        return(RIGHT);
                    case RIGHT:
                        return(LEFT);
                    default:
                        HUMOTO_THROW_MSG("Non-invertible type.");
                }
            }
    };


    /**
     * @brief Container for two symmetric objects
     *
     * @tparam t_Data type of the objects
     */
    template<typename t_Data>
        class HUMOTO_LOCAL LeftRightContainer
    {
        private:
            t_Data data[LeftOrRight::COUNT];

        public:
            /**
             * @brief Access the element
             *
             * @param[in] left_or_right
             *
             * @return data object
             */
            t_Data & operator[](const LeftOrRight::Type left_or_right)
            {
                HUMOTO_ASSERT((left_or_right == LeftOrRight::LEFT) || (left_or_right == LeftOrRight::RIGHT), "Must be left or right.");
                return(data[left_or_right - 1]);
            }


            /**
             * @copydoc operator[]
             */
            const t_Data & operator[](const LeftOrRight::Type left_or_right) const
            {
                HUMOTO_ASSERT((left_or_right == LeftOrRight::LEFT) || (left_or_right == LeftOrRight::RIGHT), "Must be left or right.");
                return(data[left_or_right - 1]);
            }


            /**
             * @brief Get/set/copy left or right object
             *
             * @return data object
             */
            t_Data & getLeft()                      { return(operator[](LeftOrRight::LEFT)); }

            /// @copydoc getLeft
            const t_Data & getLeft() const          { return(operator[](LeftOrRight::LEFT)); }

            /// @copydoc getLeft
            t_Data & getRight()                     { return(operator[](LeftOrRight::RIGHT)); }

            /// @copydoc getLeft
            const t_Data & getRight() const         { return(operator[](LeftOrRight::RIGHT)); }

            /// @copydoc getLeft
            void setLeft(const t_Data &data)        { operator[](LeftOrRight::LEFT) = data; }

            /// @copydoc getLeft
            void setRight(const t_Data &data)       { operator[](LeftOrRight::RIGHT) = data; }

            /// @copydoc getLeft
            void copyLeft(const LeftRightContainer<t_Data> &copy_from)   { operator[](LeftOrRight::LEFT) = copy_from[LeftOrRight::LEFT]; }

            /// @copydoc getLeft
            void copyRight(const LeftRightContainer<t_Data> &copy_from)  { operator[](LeftOrRight::RIGHT) = copy_from[LeftOrRight::RIGHT]; }
    };
}
