(define (domain cob-test-domain-01)
	;;;(:requirements :strips :typing :adl :open-world :procedural-attachment :universal-preconditions)
	(:requirements :strips :typing :adl )
	(:types room loc trash-bin - location
		
		box agent room-clean-tasks dirt - phys-obj
		robot user - agent
		vacuum - vacuum-cleaner
		;;;rooms-inspected - booly
		;;;trash-bin dirt - room-clean-tasks	
	)
	(:predicates
		(trans ?location-1 ?location-2 - room)
		(at ?what - phys-obj ?location - room)
		;;;(object-is-at ?what ?where) 
		;;;(have ?who ?what) 
		(is-robo ?who - robot)
		;;;(occupied ?where)
		;;;(see ?who ?what)
		(is-user ?who - user)
		;;;(neglected ?who)
		(inspected ?where - room)
		(cleared ?trash - trash-bin)
		(cleaned ?dirt - dirt)
		(reached-trash ?trash - trash-bin)
		(all-rooms-inspected ?who - robot)
		(ready-for-dirtmap ?who - robot)
		(ready-for-vacuum ?who - robot)
		(detect-trash-bins-on ?where - room)
		(detect-dirt-on ?where - room)
		;;; gripper
		;;;(gripper ?x)
		;;;(free ?x)
		;;;(carry ?what ?withwhat)
	)

	(:action change-tool
		:parameters (?who - robot)
		:precondition (and 
				 (forall (?ro - room) (inspected ?ro))
				)
		:effect (and
			    (ready-for-dirtmap ?who)
			)
	)
	
	(:action get-dirt-map
		:parameters (?who - robot)
		:precondition (and
				 (ready-for-dirtmap ?who)
			      )
		:effect (and
			     (ready-for-vacuum ?who)
			)
	)

	(:action clean-location
		:parameters (?who - robot ?where - location ?dirt - dirt)
		:precondition (and
				 (ready-for-vacuum ?who)
				 (at ?who ?where)
				 (at ?dirt ?where)
				)
		:effect (and (cleaned ?dirt))
	)

	(:action inspect-room
		:parameters (?who - robot ?where - room)
		:precondition (and
				   ;;;(is-robo ?who)
				   (at ?who ?where)
				   (not (inspected ?where))
				   (detect-trash-bins-on ?where)
				   (detect-dirt-on ?where)
				)	
		:effect (and (inspected ?where))
	)
	
	(:action move-to-trash-bin
		:parameters (?who - robot ?where - room ?trash - trash-bin)
		:precondition (and
				 (at ?who ?where)
				 (at ?trash ?where)
				 (inspected ?where)
				 (not (cleared ?trash))
				)
		:effect (and (reached-trash ?trash)
			)
	)

	(:action clear-trash-bin
		:parameters (?who - robot ?where - room ?trash - trash-bin)
		:precondition (and
				 (at ?who ?where)
			         (at ?trash ?where)
				 (inspected ?where)
				 (reached-trash ?trash)
				 (not (cleared ?trash))
				)
		:effect (and (cleared ?trash)
			)
	)
	
	(:action detect-trash-bins-off
		:parameters (?who - robot ?where - room)
		:precondition (and
				   (at ?who ?where)
				   (detect-trash-bins-on ?where)
				)
		:effect (and (not (detect-trash-bins-on ?where))
			)
	)

	(:action detect-trash-bins-on
		:parameters (?who - robot ?where - room)
		:precondition (and
				   (at ?who ?where)
				   (not (inspected ?where))
				   (not (detect-trash-bins-on ?where))
				)
		:effect (and (detect-trash-bins-on ?where))
	)

	(:action detect-dirt-off
		:parameters (?who - robot ?where - room)
		:precondition (and
				   (at ?who ?where)
				   (detect-dirt-on ?where)
				)
		:effect (and (not (detect-dirt-on ?where))
			)
	)
	
	(:action detect-dirt-on
		:parameters (?who - robot ?where - room)
		:precondition (and
				  (at ?who ?where)
				  (not (inspected ?where))
				  (not (detect-dirt-on ?where))
			      )
		:effect (and (detect-dirt-on ?where))
	)

	(:action move-to
		:parameters (?who - agent ?from ?to - room)
		:precondition (and 
				   ;;;(is-robo ?who)
				   (at ?who ?from)
				   (trans ?from ?to)
				   ;;;(not (occupied ?to))
				   ;;;(neglected ?who)
				   (not (detect-dirt-on ?from))
				   (not (detect-trash-bins-on ?from))
				   )
		:effect (and (not (at ?who ?from)
				  )
			     (at ?who ?to)
			     ;;;(neglected ?who)
			     )
	)
	
	;;;(:action move-user-to
	;;;	:parameters (?who - user ?from ?to - room)
	;;;	:precondition (and
	;;;			 (is-user ?who)
	;;;			 (at ?who ?from)
	;;;			 (trans ?from ?to)
	;;;			 ;;;(not (occupied ?to))
	;;;		      )
	;;;	:effect (and (not (at ?who ?from))
	;;;		     (at ?who ?to)
	;;;		)
	;;;)

	;;;room cleaning:
	;;;(:action dirt-detection-on
	;;;	:parameters (
	;;;(:action look-at
	;;;	:parameters (?who ?what ?where ?close-to)
	;;;	:precondition (and (is-robo ?who)
	;;;			   (at ?who ?where)
	;;;			   (trans ?where ?close-to)
	;;;			   (at ?what ?close-to)
	;;;			   (neglected ?who)
	;;;			   )
	;;;	:effect (and (see ?who ?what)
	;;;		     (not (neglected ?who))
	;;;		)
	;;;)

	;;;(:action grip-it
	;;;	:parameters (?who ?what ?where ?from ?withwhat)
	;;;	:precondition (and (is-robo ?who)
	;;;			   (at ?who ?where)
	;;;			   (see ?who ?what)
	;;;			   (trans ?where ?from)
	;;;			   (at ?what ?from)
	;;;			   (not (neglected ?who))
	;;;
	;;;			   ;;; gripper
	;;;			   (gripper ?withwhat)
	;;;			   (free ?withwhat)
	;;;			   )
	;;;	:effect (and 	(neglected ?who)
	;;;			;;;(have ?who ?what)
	;;;			(carry ?what ?withwhat)
	;;;			(not (occupied ?from))
	;;;			(not (at ?what ?from))
	;;;			(not (see ?who ?what))
	;;;			(not (free ?withwhat))
	;;;		)
	;;;)
	
	;;;(:action put-it
	;;;	:parameters (?who ?what ?where ?to ?withwhat)
	;;;	:precondition (and (is-robo ?who)
	;;;			   ;;;(is-user ?pers)
	;;;			   (at ?who ?where)
	;;;			   ;;;(have ?who ?what)
	;;;			   (trans ?where ?to)
	;;;			   ;;;(at ?pers ?where)
	;;;			   (gripper ?withwhat)
	;;;			   (carry ?what ?withwhat)
	;;;			)
	;;;	:effect (and (at ?what ?to)
	;;;		      (occupied ?to)
	;;;		      ;;;(not (have ?who ?what))
	;;;		      (free ?withwhat)
	;;;		      (not (carry ?what ?withwhat))
	;;;		)
	;;;)
	
	;;;(:action deliver-to
	;;;	:parameters (?pers ?what ?who ?where ?to ?withwhat)
	;;;	:precondition (and (is-robo ?who)
	;;;			   (is-user ?pers)
	;;;			   ;;;(have ?who ?what)
	;;;			   (at ?pers ?to)
	;;;			   (at ?who ?where)
	;;;			   (trans ?where ?to)
	;;;			   (see ?who ?pers)
	;;;			   (not (neglected ?who))
	;;;			   (gripper ?withwhat)
	;;;			   (carry ?what ?withwhat)
	;;;			)
	;;;	:effect (and (have ?pers ?what)
	;;;		     (neglected ?who)
	;;;		     ;;;(not (have ?who ?what))
	;;;		     (not (see ?who ?what))
	;;;		     (free ?withwhat)
	;;;		     (carry ?what ?withwhat)
	;;;		)
	;;;)

	;;;(:action neglect
	;;;	:parameters (?pers ?who)
	;;;	:precondition (and (is-robo ?who)
	;;;			   (is-user ?pers)
	;;;				)
	;;;	:effect (and (neglected ?who)
	;;;		     (not (see ?who ?pers))
	;;;		)
	;;;)

)
