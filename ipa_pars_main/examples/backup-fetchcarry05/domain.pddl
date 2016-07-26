(define (domain cob-test-domain-01)
	(:requirements :strips :typing)
	(:types room phys-obj gripper
		box agent - phys-obj
		robot user - agent
	)
	(:predicates
		(trans ?location-1 ?location-2 - room)
		(at ?what - phys-obj ?location - room)
		(have ?who - agent ?what - phys-obj)
		(occupied ?where - room)
	(see ?who - robot ?what - phys-obj)
	(neglected ?who - robot)

	;;; gripper
	(which-gripper ?x - gripper)
	(gripper-free ?x - gripper)
	(carry ?what - phys-obj ?withwhat - gripper)
	)

	(:action move-robo-to
		:parameters (?who - robot ?from - room ?to - room)
		:precondition (and 
				   (at ?who ?from)
				   (trans ?from ?to)
				   (not (occupied ?to))
				   (neglected ?who)
				   )
		:effect (and (not (at ?who ?from))
			     (at ?who ?to)
			     (neglected ?who)
			     )
	)
	
	(:action look-at
		:parameters (?who - robot ?what - phys-obj ?where - room ?close-to - room)
		:precondition (and
				   (at ?who ?where)
				   (trans ?where ?close-to)
				   (at ?what ?close-to)
				   (neglected ?who)
				   )
		:effect (and (see ?who ?what)
			     (not (neglected ?who))
			)
	)

	(:action grip-it
		:parameters (?who - robot ?what - phys-obj ?where - room ?from - room ?withwhat - gripper)
		:precondition (and 
				   (at ?who ?where)
				   (see ?who ?what)
				   (trans ?where ?from)
				   (at ?what ?from)
				   (not (neglected ?who))
	
				   ;;; gripper
				   (which-gripper ?withwhat)
				   (gripper-free ?withwhat)
				   )
		:effect (and 	(neglected ?who)
				
				(carry ?what ?withwhat)
				(not (occupied ?from))
				(not (at ?what ?from))
				(not (see ?who ?what))
				(not (gripper-free ?withwhat))
			)
	)
	
	(:action put-it
		:parameters (?who - robot ?what - phys-obj ?where - room ?to - room ?withwhat - gripper)
		:precondition (and ;;;(is-robo ?who)
				   ;;;(is-user ?pers)
				   (at ?who ?where)
				   ;;;(have ?who ?what)
				   (trans ?where ?to)
				   ;;;(at ?pers ?where)
				   (which-gripper ?withwhat)
				   (carry ?what ?withwhat)
				)
		:effect (and (at ?what ?to)
			      (occupied ?to)
			      ;;;(not (have ?who ?what))
			      (gripper-free ?withwhat)
			      (not (carry ?what ?withwhat))
			)
	)
	
	(:action deliver-to
		:parameters (?pers - user ?what - phys-obj ?who - robot ?where - room ?to - room ?withwhat - gripper)
		:precondition (and
				   (at ?pers ?to)
				   (at ?who ?where)
				   (trans ?where ?to)
				   (see ?who ?pers)
				   (not (neglected ?who))
				   (which-gripper ?withwhat)
				   (carry ?what ?withwhat)
				)
		:effect (and (have ?pers ?what)
			     (neglected ?who)
			     (not (see ?who ?what))
			     (gripper-free ?withwhat)
			     (carry ?what ?withwhat)
			)
	)

	(:action neglect
		:parameters (?pers - user ?who - robot )
		:precondition (and
				   (not (neglected ?who))
					)
		:effect (and (neglected ?who)
			     (not (see ?who ?pers))
			)
	)

)
