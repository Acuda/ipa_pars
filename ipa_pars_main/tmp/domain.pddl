(define (domain cob-test-domain-01)
 	(:requirements :strips)
 	(:predicates
 		(trans ?location-1 ?location-2)
 		(at ?what ?location)
 		(object-is-at ?what ?where) 
 		(have ?who ?what) 
 		(is-robo ?who)
 		(see ?who ?what)
 		(near ?who ?where)
 	)
 
 	(:action move-robo-to
 		:parameters (?who ?from ?to)
 		:precondition (and 
 				   (is-robo ?who)
 				   (at ?who ?from)
 				   (trans ?from ?to))
 		:effect (and (not (at ?who ?from))
 			     (at ?who ?to))
 	)
 
 	;;;(:action take
 	;;;	:parameters (?who ?what ?where)
 	;;;	:precondition (and (is-robo ?who)
 	;;;			   (at ?who ?where)
 	;;;			   (at ?what ?where)
 	;;;			   (see ?who ?what))
 	;;;
 	;;;	:effect (and (have ?who ?what)
 	;;;		     (not (at ?what ?where)))
 	;;;)
 
 	(:action look-at
 		:parameters (?who ?what ?where ?to)
 		:precondition (and (is-robo ?who)
 				   (at ?who ?where)
 				   (trans ?where ?to)
 				   (at ?what ?to)
 				   )
 		:effect (and (see ?who ?what))
 	)
 
 	(:action grip-it
 		:parameters (?who ?what ?where ?to)
 		:precondition (and (is-robo ?who)
 				   (at ?who ?where)
 				   (see ?who ?what)
 				   (trans ?where ?to)
 				   )
 		:effect (and (have ?who ?what)
 				(not (at ?what ?to)))
 	)
 
 )
