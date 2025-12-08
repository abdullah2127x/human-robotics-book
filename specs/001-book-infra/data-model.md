# Data Model: Book Infrastructure

## Overview

This data model describes the structure for the Physical AI & Humanoid Robotics book infrastructure using Docusaurus. Since this is a static documentation site, the "data model" primarily refers to the content organization and navigation structure rather than traditional database entities.

## Content Structure

### Book Modules
- **Entity**: BookModule
- **Fields**:
  - id: string (unique identifier)
  - title: string (display name)
  - description: string (brief description)
  - order: number (sequence in the book)
  - pages: Page[] (collection of pages in the module)
- **Relationships**: One-to-many with Page entities
- **Validation**: Must have unique id and non-empty title
- **State**: Active (visible to users)

### Pages
- **Entity**: Page
- **Fields**:
  - id: string (unique identifier)
  - title: string (page title)
  - content: string (markdown content)
  - module: BookModule (parent module)
  - order: number (sequence within module)
  - slug: string (URL-friendly identifier)
  - metadata: object (additional page metadata)
- **Relationships**: Many-to-one with BookModule
- **Validation**: Must have unique slug within module, non-empty title
- **State**: Draft | Published (controlling visibility)

### Navigation Items
- **Entity**: NavigationItem
- **Fields**:
  - id: string (unique identifier)
  - label: string (display text)
  - href: string (destination URL)
  - type: string (link | dropdown | doc)
  - position: string (left | right, for navbar positioning)
  - children: NavigationItem[] (for dropdowns)
- **Relationships**: Hierarchical through children
- **Validation**: Must have either href or children (not both)
- **State**: Enabled | Disabled (controlling visibility)

## Configuration Objects

### Site Configuration
- **Entity**: SiteConfig
- **Fields**:
  - title: string (site title)
  - tagline: string (site tagline)
  - url: string (base URL)
  - baseUrl: string (base path)
  - favicon: string (path to favicon)
  - organizationName: string (GitHub org/user name)
  - projectName: string (GitHub repo name)
  - deploymentBranch: string (branch for GitHub Pages)
  - i18n: object (internationalization config)
- **Validation**: URL must be valid, baseUrl must start with "/"
- **State**: Active (single instance)

### Theme Configuration
- **Entity**: ThemeConfig
- **Fields**:
  - navbar: object (navigation bar settings)
  - footer: object (footer settings)
  - prism: object (code block settings)
  - colorMode: object (light/dark mode settings)
- **Validation**: Must conform to Docusaurus theme schema
- **State**: Active (single instance)

## Sidebar Structure

### Sidebar Category
- **Entity**: SidebarCategory
- **Fields**:
  - type: string (always "category")
  - label: string (category name)
  - items: array (collection of links/doc references)
  - collapsed: boolean (initial collapsed state)
  - collapsible: boolean (whether it can be collapsed)
- **Relationships**: Contains multiple SidebarItem entities
- **Validation**: Must have a label and non-empty items array
- **State**: Collapsed | Expanded

### Sidebar Item
- **Entity**: SidebarItem
- **Fields**:
  - type: string ("doc" | "link" | "ref")
  - id: string (document id for "doc" type)
  - label: string (display text)
  - href: string (URL for "link" type)
- **Validation**: Must have appropriate fields based on type
- **State**: Visible | Hidden

## Validation Rules

1. **Module Uniqueness**: Each BookModule must have a unique id
2. **Page Uniqueness**: Each Page must have a unique slug within its parent module
3. **Navigation Integrity**: All navigation links must point to valid destinations
4. **Sidebar Completeness**: All modules must be represented in the sidebar structure
5. **Content Requirements**: All pages must have appropriate titles and content
6. **Accessibility Compliance**: All content must meet WCAG 2.1 AA standards

## State Transitions

### Page State Transitions
- Draft → Published (when content is ready for public consumption)
- Published → Draft (when content needs editing)

### Module State Transitions
- Planned → Active (when module content is being developed)
- Active → Complete (when module content is finalized)

## Relationships

```
SiteConfig (1) ←→ (1) ThemeConfig
SiteConfig (1) ←→ (*) BookModule
BookModule (1) ←→ (*) Page
BookModule (1) ←→ (*) SidebarCategory
NavigationItem (1) ←→ (*) NavigationItem (children)
SidebarCategory (1) ←→ (*) SidebarItem
```

## Constraints

1. Maximum 4 main book modules (Intro, Module 1-3) as per specification
2. All modules must be represented in the sidebar
3. Navigation must include Learn, Resources, Capstone, and Login/Personalize links
4. All content must be accessible (WCAG 2.1 AA compliance)
5. Site must load within 3 seconds on 3G connection
6. All pages must be keyboard navigable
7. All images must have appropriate alt text