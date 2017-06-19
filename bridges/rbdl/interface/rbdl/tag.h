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
    namespace rbdl
    {
        typedef std::size_t     LinkId;


        /**
         * @brief Tag base class
         */
        class HUMOTO_LOCAL TagBase
        {
            protected:
                ~TagBase() {};
        };


        /**
         * @brief Link tag
         */
        class HUMOTO_LOCAL TagLink : public TagBase
        {
            public:
                LinkId    link_id_;


            public:
                TagLink(const LinkId      & link_id)
                {
                    link_id_ = link_id;
                }
        };
        typedef boost::shared_ptr<const TagLink>  TagLinkPtr;



        /**
         * @brief Link point tag
         */
        class HUMOTO_LOCAL TagPoint : public TagBase
        {
            public:
                LinkId          link_id_;
                etools::Vector3 local_position_;


            public:
                TagPoint(   const LinkId &        link_id,
                            const etools::Vector3 & local_position)
                {
                    link_id_ = link_id;
                    local_position_ = local_position;
                }
        };
        typedef boost::shared_ptr<const TagPoint>  TagPointPtr;


        /**
         * @brief CoM tag
         */
        class HUMOTO_LOCAL TagCoM : public TagBase
        {
            public:
                TagCoM()
                {
                }
        };
        typedef boost::shared_ptr<const TagCoM>  TagCoMPtr;


        /**
         * @brief Partial CoM tag (subset of links)
         */
        class HUMOTO_LOCAL TagPartialCoM : public TagBase
        {
            public:
                std::vector<LinkId>         link_ids_;
                double                      mass_;

            public:
                TagPartialCoM(const std::vector<LinkId> &   link_ids,
                              const double                  mass)
                {
                    link_ids_ = link_ids;
                    mass_ = mass;
                }
        };
        typedef boost::shared_ptr<const TagPartialCoM>  TagPartialCoMPtr;
    }
}
