/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NOTE: This file has been modified by Sony Corporation.
 * Modifications are Copyright 2021 Sony Corporation,
 * and licensed under the license of the file.
 */
void convert_to_tag_list(struct tag *tags);

#ifdef CONFIG_ATAGS
const struct machine_desc *setup_machine_tags(void *__atags_vaddr,
	unsigned int machine_nr);
#else
static inline const struct machine_desc * __init __noreturn
setup_machine_tags(void *__atags_vaddr, unsigned int machine_nr)
{
	early_print("no ATAGS support: can't continue\n");
	while (true);
	unreachable();
}
#endif
