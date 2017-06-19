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
     * @brief Abstract class to be used for interfaces.
     */
    class HUMOTO_LOCAL ModelState
    {
        protected:
            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~ModelState() {}
            ModelState() {}


        public:
            virtual void    log(humoto::Logger &, const LogEntryName &, const std::string &) const = 0;
    };



    /**
     * @brief Instances of this class are passed to a virtual method '@ref
     * humoto::TaskBase::form()', so even though this class is basically
     * useless in its present form we cannot avoid its definition using a
     * template.
     */
    class HUMOTO_LOCAL Model
    {
        protected:
            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~Model() {}
            Model() {}


        public:
            virtual void    updateState(const humoto::ModelState &) = 0;

            virtual void    log(humoto::Logger &, const LogEntryName &, const std::string &) const = 0;

            void noop() const
            {
            }
    };
}
